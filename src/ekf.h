#pragma once

// Error-state Kalman filter interface. 15-state layout per ekf_state.h.
//
// IMU-only observability (see CLAUDE.md): roll/pitch and gyro bias x/y
// converge; yaw, position and velocity drift without aiding. Aiding
// sensors are added as new update_*() member functions, not a rewrite.
//
// Usage cycle, all in SI units and the NED frame:
//   predict(f, w, dt)      every IMU sample
//   update_accel(f)        at a lower rate, gated internally
//   update_zupt()          when the caller has detected standstill
// Each update computes dx and immediately calls inject(), so the error
// state is always zero between calls.

#include "ekf_state.h"

namespace ekf {

// Continuous-time noise densities and measurement variances. Tunables --
// starting values are typical MPU6050 figures; expect to retune on the
// vehicle with vibration present.
struct NoiseParams {
    Scalar sigma_acc  = 0.06f;    // accel white noise      (m/s^2 /sqrt(Hz))
    Scalar sigma_gyro = 0.003f;   // gyro white noise       (rad/s /sqrt(Hz))
    Scalar sigma_ba_walk = 0.01f; // accel bias random walk (m/s^3 /sqrt(Hz))
    Scalar sigma_bg_walk = 1e-4f; // gyro bias random walk  (rad/s^2 /sqrt(Hz))

    Scalar accel_meas_var = 0.25f;  // R for the gravity update ((m/s^2)^2)
    Scalar zupt_meas_var  = 0.01f;  // R for zero-velocity     ((m/s)^2)

    // Gravity-vector update gates.
    Scalar accel_norm_gate = 1.0f;  // reject if | |f| - g | exceeds (m/s^2)
    Scalar nis_gate = 7.815f;       // chi-square, 3 dof, 95%
};

class Filter {
public:
    // Reset state and covariance to startup values. Attitude/bias must be
    // re-seeded afterwards (init_attitude_from_accel / set_gyro_bias).
    void reset();

    // Seed roll/pitch from a stationary specific-force sample (yaw = 0,
    // unobservable anyway).
    void init_attitude_from_accel(const Vec3& f_b);
    void set_gyro_bias(const Vec3& bg);

    // Propagate nominal state and covariance by one IMU sample.
    // Returns false (and does nothing) if dt is outside (0, 0.1].
    bool predict(const Vec3& f_meas, const Vec3& w_meas, Scalar dt);

    // Gravity-vector attitude update. Returns false if gated out.
    bool update_accel(const Vec3& f_meas);

    // Zero-velocity update; call only while stationary.
    bool update_zupt();

    // All state and covariance entries finite. On false, caller should
    // reset() and re-seed -- NaN is contagious and permanent.
    bool healthy() const;

    const Nominal& state() const { return x_; }
    const Covariance& covariance() const { return P_; }

    Vec3 rpy() const;  // roll, pitch, yaw (rad), from the nominal quaternion

    uint32_t accel_updates_gated = 0;  // diagnostics
    uint32_t accel_updates_applied = 0;

    NoiseParams noise;

private:
    // Shared tail of every measurement update: innovation covariance,
    // optional NIS gate (chi-square threshold; <= 0 disables), gain,
    // Joseph-form P update, inject. H is 3x15, y the 3x1 innovation,
    // r_var the (isotropic) measurement variance. Returns false if gated.
    bool apply_update(const Eigen::Matrix<Scalar, 3, kN>& H, const Vec3& y,
                      Scalar r_var, Scalar nis_gate);

    Nominal x_;
    ErrorState dx_ = ErrorState::Zero();
    Covariance P_ = Covariance::Identity();

    // Workspace for 15x15 matrices, kept out of the stack (members, not
    // function locals -- 15*15*4 = 900 bytes each).
    StateMat phi_;
    StateMat tmp_;
    StateMat joseph_;
};

}  // namespace ekf
