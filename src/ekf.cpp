// Error-state Kalman filter implementation. Pure math, no hardware --
// compiles unchanged on the desktop for replay testing.

#include "ekf.h"
#include "ekf_predict.h"

#include <cmath>

namespace ekf {

void Filter::reset()
{
    x_ = Nominal{};
    dx_.setZero();

    P_.setZero();
    P_.diagonal().segment<3>(kPos).setConstant(1e-4f);   // origin is defined here
    P_.diagonal().segment<3>(kVel).setConstant(1e-2f);   // starts at rest
    P_.diagonal().segment<3>(kAtt).setConstant(9e-3f);   // ~5 deg roll/pitch seed
    P_(kAtt + 2, kAtt + 2) = 0.3f;                       // yaw: unobservable, keep loose
    P_.diagonal().segment<3>(kBAcc).setConstant(4e-2f);  // (0.2 m/s^2)^2
    P_.diagonal().segment<3>(kBGyr).setConstant(1e-4f);  // (0.01 rad/s)^2

    accel_updates_gated = 0;
    accel_updates_applied = 0;
}

void Filter::init_attitude_from_accel(const Vec3& f_b)
{
    // Stationary: f_b = -R^T g_n, so with ZYX Euler angles and yaw = 0,
    //   f = g * [sin(pitch), -sin(roll)cos(pitch), -cos(roll)cos(pitch)]
    const Scalar roll  = atan2f(-f_b.y(), -f_b.z());
    const Scalar pitch = atan2f(f_b.x(),
                                sqrtf(f_b.y() * f_b.y() + f_b.z() * f_b.z()));

    x_.q = Eigen::AngleAxis<Scalar>(pitch, Vec3::UnitY()) *
           Eigen::AngleAxis<Scalar>(roll, Vec3::UnitX());
    x_.q.normalize();
}

void Filter::set_gyro_bias(const Vec3& bg) { x_.bg = bg; }

bool Filter::predict(const Vec3& f_meas, const Vec3& w_meas, Scalar dt)
{
    if (!(dt > 0.0f) || dt > 0.1f) return false;  // also rejects NaN dt

    // Phi is linearized at the pre-propagation state.
    error_transition(x_, f_meas, w_meas, dt, phi_);
    propagate_nominal(x_, f_meas, w_meas, dt);

    // P = Phi P Phi^T + Qd. Two steps through tmp_: Eigen aliases if a
    // matrix appears on both sides of a product assignment.
    tmp_.noalias() = phi_ * P_;
    P_.noalias() = tmp_ * phi_.transpose();

    // Additive discrete process noise (diagonal). Position takes its
    // noise through the velocity coupling in Phi.
    const NoiseParams& n = noise;
    P_.diagonal().segment<3>(kVel)
        .array() += n.sigma_acc * n.sigma_acc * dt;
    P_.diagonal().segment<3>(kAtt)
        .array() += n.sigma_gyro * n.sigma_gyro * dt;
    P_.diagonal().segment<3>(kBAcc)
        .array() += n.sigma_ba_walk * n.sigma_ba_walk * dt;
    P_.diagonal().segment<3>(kBGyr)
        .array() += n.sigma_bg_walk * n.sigma_bg_walk * dt;

    // Force symmetry -- float32 round-off skews P a little every cycle.
    tmp_ = P_.transpose();
    P_ += tmp_;
    P_ *= 0.5f;

    return true;
}

bool Filter::update_accel(const Vec3& f_meas)
{
    // The accelerometer only indicates "down" when the vehicle is not
    // accelerating; gate on the gravity norm first.
    if (fabsf(f_meas.norm() - kGravity) > noise.accel_norm_gate) {
        accel_updates_gated++;
        return false;
    }

    const Mat3 Rt = x_.q.toRotationMatrix().transpose();  // NED -> body
    const Vec3 g_body = Rt * gravity_ned();

    // Measurement model: z = -R^T g_n + ba + noise.
    const Vec3 y = f_meas - (x_.ba - g_body);

    Eigen::Matrix<Scalar, 3, kN> H = Eigen::Matrix<Scalar, 3, kN>::Zero();
    H.block<3, 3>(0, kAtt) = -skew(g_body);
    H.block<3, 3>(0, kBAcc) = Mat3::Identity();

    if (!apply_update(H, y, noise.accel_meas_var, noise.nis_gate)) {
        accel_updates_gated++;
        return false;
    }
    accel_updates_applied++;
    return true;
}

bool Filter::update_zupt()
{
    // z = 0 = v + noise, so the innovation is just -v_hat.
    Eigen::Matrix<Scalar, 3, kN> H = Eigen::Matrix<Scalar, 3, kN>::Zero();
    H.block<3, 3>(0, kVel) = Mat3::Identity();
    return apply_update(H, -x_.v, noise.zupt_meas_var, 0.0f);
}

bool Filter::apply_update(const Eigen::Matrix<Scalar, 3, kN>& H,
                          const Vec3& y, Scalar r_var, Scalar nis_gate)
{
    const Eigen::Matrix<Scalar, 3, kN> HP = H * P_;
    Mat3 S = HP * H.transpose();
    S.diagonal().array() += r_var;
    const Mat3 S_inv = S.inverse();  // closed-form for 3x3, no allocation

    if (nis_gate > 0.0f && y.dot(S_inv * y) > nis_gate) return false;

    const Eigen::Matrix<Scalar, kN, 3> K = HP.transpose() * S_inv;
    dx_ = K * y;

    // Joseph form: P = (I-KH) P (I-KH)^T + K R K^T. The simple
    // (I-KH)P form loses symmetry/positivity in float32 and diverges.
    joseph_.setIdentity();
    joseph_.noalias() -= K * H;
    tmp_.noalias() = joseph_ * P_;
    P_.noalias() = tmp_ * joseph_.transpose();
    P_.noalias() += r_var * (K * K.transpose());

    inject(x_, dx_);  // folds dx into the nominal state, zeroes dx
    return true;
}

bool Filter::healthy() const
{
    return P_.allFinite() && x_.q.coeffs().allFinite() && x_.p.allFinite() &&
           x_.v.allFinite() && x_.ba.allFinite() && x_.bg.allFinite();
}

Vec3 Filter::rpy() const
{
    const Quat& q = x_.q;
    const Scalar w = q.w(), x = q.x(), y = q.y(), z = q.z();

    const Scalar roll = atan2f(2.0f * (w * x + y * z),
                               1.0f - 2.0f * (x * x + y * y));
    Scalar sp = 2.0f * (w * y - z * x);
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    const Scalar pitch = asinf(sp);
    const Scalar yaw = atan2f(2.0f * (w * z + x * y),
                              1.0f - 2.0f * (y * y + z * z));
    return Vec3(roll, pitch, yaw);
}

}  // namespace ekf
