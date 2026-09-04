#pragma once

// Predict-step math for the error-state KF: nominal-state propagation and
// the discrete error-state transition matrix. Pure functions of the state
// and one IMU sample; no hardware, no globals, desktop-testable.
//
// Frame: NED. Gravity is [0, 0, +9.80665] (Z down). The accelerometer
// measures specific force f_b = R^T (a_n - g_n), so a_n = R f_b + g_n.

#include "ekf_state.h"

namespace ekf {

inline constexpr Scalar kGravity = 9.80665f;

inline Vec3 gravity_ned() { return Vec3(0.0f, 0.0f, kGravity); }

// Advance the nominal state by one IMU sample over dt.
// f_meas / w_meas are the raw specific force (m/s^2) and angular rate
// (rad/s) in the body frame; biases are removed here using the state's
// current estimates.
inline void propagate_nominal(Nominal& s, const Vec3& f_meas,
                              const Vec3& w_meas, Scalar dt)
{
    const Vec3 f_b = f_meas - s.ba;
    const Vec3 w_b = w_meas - s.bg;

    const Mat3 R = s.q.toRotationMatrix();       // body -> NED
    const Vec3 a_n = R * f_b + gravity_ned();    // NED acceleration

    s.p += s.v * dt + 0.5f * a_n * dt * dt;
    s.v += a_n * dt;
    s.q = (s.q * deltaQ(w_b * dt)).normalized();
}

// Discrete error-state transition Phi ~= I + F*dt (first order), for the
// same sample that was fed to propagate_nominal (bias-corrected here the
// same way). Written into `Phi` in place -- the 15x15 stays in the
// caller's storage, per the no-stack-largesse rule.
//
// Non-zero blocks of F (standard ESKF, right-multiplied attitude error):
//   d(dp)/d(dv)     =  I
//   d(dv)/d(dtheta) = -R * skew(f_b)
//   d(dv)/d(dba)    = -R
//   d(dtheta)/d(dtheta) = -skew(w_b)
//   d(dtheta)/d(dbg)    = -I
inline void error_transition(const Nominal& s, const Vec3& f_meas,
                             const Vec3& w_meas, Scalar dt, StateMat& Phi)
{
    const Vec3 f_b = f_meas - s.ba;
    const Vec3 w_b = w_meas - s.bg;
    const Mat3 R = s.q.toRotationMatrix();
    const Mat3 I3 = Mat3::Identity();

    Phi.setIdentity();
    Phi.block<3, 3>(kPos, kVel)  =  I3 * dt;
    Phi.block<3, 3>(kVel, kAtt)  = -R * skew(f_b) * dt;
    Phi.block<3, 3>(kVel, kBAcc) = -R * dt;
    Phi.block<3, 3>(kAtt, kAtt)  =  I3 - skew(w_b) * dt;
    Phi.block<3, 3>(kAtt, kBGyr) = -I3 * dt;
}

}  // namespace ekf
