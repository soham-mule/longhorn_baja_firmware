#pragma once

// Arduino's F() flash-string macro collides with Eigen's use of `F` as a
// template parameter name (the preprocessor rewrites it everywhere).
// Neutralize it around the Eigen includes and restore Arduino's
// definition afterwards, so include order doesn't matter.
#ifdef F
#undef F
#define EKF_RESTORE_ARDUINO_F
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifdef EKF_RESTORE_ARDUINO_F
#undef EKF_RESTORE_ARDUINO_F
#define F(string_literal) \
    (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))
#endif

namespace ekf {

using Scalar = float;

using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
using Quat = Eigen::Quaternion<Scalar>;

// Layout of the 15-element error state. Frame is NED.
enum Idx : int {
    kPos  = 0,   // position error        (m)
    kVel  = 3,   // velocity error        (m/s)
    kAtt  = 6,   // attitude error        (rotation vector, rad)
    kBAcc = 9,   // accelerometer bias    (m/s^2)
    kBGyr = 12,  // gyroscope bias        (rad/s)
    kN    = 15
};

using ErrorState = Eigen::Matrix<Scalar, kN, 1>;   // dx
using Covariance = Eigen::Matrix<Scalar, kN, kN>;  // P
using StateMat   = Eigen::Matrix<Scalar, kN, kN>;  // F, Phi

// Nominal state: 16 parameters, because attitude is a quaternion here.
// It is propagated directly by the IMU and never appears in dx.
struct Nominal {
    Vec3 p  = Vec3::Zero();
    Vec3 v  = Vec3::Zero();
    Quat q  = Quat::Identity();
    Vec3 ba = Vec3::Zero();
    Vec3 bg = Vec3::Zero();
};

// Named views into dx. These return Eigen blocks that alias the original
// vector, so they read and assign in place -- no copy.
inline auto pos (ErrorState& x) { return x.segment<3>(kPos);  }
inline auto vel (ErrorState& x) { return x.segment<3>(kVel);  }
inline auto att (ErrorState& x) { return x.segment<3>(kAtt);  }
inline auto bacc(ErrorState& x) { return x.segment<3>(kBAcc); }
inline auto bgyr(ErrorState& x) { return x.segment<3>(kBGyr); }

inline auto pos (const ErrorState& x) { return x.segment<3>(kPos);  }
inline auto vel (const ErrorState& x) { return x.segment<3>(kVel);  }
inline auto att (const ErrorState& x) { return x.segment<3>(kAtt);  }
inline auto bacc(const ErrorState& x) { return x.segment<3>(kBAcc); }
inline auto bgyr(const ErrorState& x) { return x.segment<3>(kBGyr); }

// Skew-symmetric matrix, for building F and the attitude update.
inline Mat3 skew(const Vec3& w)
{
    Mat3 m;
    m <<     0, -w.z(),  w.y(),
         w.z(),      0, -w.x(),
        -w.y(),  w.x(),      0;
    return m;
}

// Small-angle rotation vector -> quaternion.
inline Quat deltaQ(const Vec3& dtheta)
{
    const Vec3 h = 0.5f * dtheta;
    return Quat(1.0f, h.x(), h.y(), h.z()).normalized();
}

// Fold the estimated error back into the nominal state, then zero it.
// Position/velocity/bias errors are additive; attitude error is not.
inline void inject(Nominal& s, ErrorState& dx)
{
    s.p  += pos(dx);
    s.v  += vel(dx);
    s.q   = (s.q * deltaQ(att(dx))).normalized();
    s.ba += bacc(dx);
    s.bg += bgyr(dx);
    dx.setZero();
}

}  // namespace ekf
