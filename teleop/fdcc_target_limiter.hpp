#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace smrcore::peripherals::detail
{
inline bool ValidTargetFeedforward(double gain)
{
    return std::isfinite(gain) && gain >= 0.0 && gain <= 1.0;
}

inline bool ValidFdccShaping(double speed, double accel, double gain, double scale)
{
    return std::isfinite(speed) && speed >= 0.01 && speed <= 0.5 &&
           std::isfinite(accel) && accel >= 0.05 && accel <= 2.0 &&
           std::isfinite(gain) && gain >= 1.0 && gain <= 40.0 &&
           std::isfinite(scale) && scale >= 1.0 && scale <= 3.0;
}

// Compute before IK/FK validation. Preserve orientation and the raw clutch
// anchor; this pose must never be treated as an already-approved target.
inline Eigen::Isometry3d MakeFdccVirtualTarget(const Eigen::Isometry3d &raw,
                                              const Eigen::Isometry3d &measured,
                                              double scale)
{
    if (!raw.matrix().allFinite() || !measured.matrix().allFinite() ||
        !std::isfinite(scale) || scale < 1.0 || scale > 3.0)
    {
        throw std::runtime_error("invalid FDCC virtual target/scale");
    }
    auto target = raw;
    const Eigen::Vector3d error = raw.translation() - measured.translation();
    const double distance = error.stableNorm();
    if (!error.allFinite() || !std::isfinite(distance))
    {
        throw std::runtime_error("invalid FDCC virtual target error");
    }
    if (distance > 0.015 && scale > 1.0)
    {
        // No boost within 15mm; smooth onset over 15..30mm, at most 30mm
        // additional lead. This can increase contact force, not cancel it.
        const double t = std::clamp((distance - 0.015) / 0.015, 0.0, 1.0);
        const double blend = t * t * (3.0 - 2.0 * t);
        const double lead = std::min((scale - 1.0) * distance * blend, 0.03);
        target.translation() += (error / distance) * lead;
    }
    if (!target.matrix().allFinite())
    {
        throw std::runtime_error("nonfinite FDCC virtual target result");
    }
    return target;
}

// Target shaping only. CM owns joint limits and the FDCC tracking-error stop.
class FdccTargetLimiter
{
public:
    explicit FdccTargetLimiter(double max_speed = 0.15, double max_accel = 0.6,
                               double follow_gain = 8.0, double feedforward = 0.0)
        : max_speed_(max_speed), max_accel_(max_accel), follow_gain_(follow_gain),
          feedforward_gain_(feedforward)
    {
        if (!ValidFdccShaping(max_speed, max_accel, follow_gain, 1.0) ||
            !ValidTargetFeedforward(feedforward))
        {
            throw std::runtime_error("invalid FDCC target shaping parameters");
        }
    }
    void Reset()
    {
        initialized_ = false;
        sent_.setIdentity();
        velocity_.setZero();
        angular_velocity_.setZero();
        target_velocity_.setZero();
        target_angular_velocity_.setZero();
        previous_target_.setIdentity();
        previous_stamp_ = 0.0;
    }
    Eigen::Isometry3d Update(const Eigen::Isometry3d &wanted,
                            const Eigen::Isometry3d &measured, double dt,
                            double target_stamp = 0.0)
    {
        if (!wanted.matrix().allFinite() || !measured.matrix().allFinite() ||
            !std::isfinite(dt) || dt < 0.0 || !std::isfinite(target_stamp))
        {
            throw std::runtime_error("nonfinite FDCC target/feedback/dt");
        }
        if (!initialized_)
        {
            sent_ = measured;
            initialized_ = true;
            previous_target_ = wanted;
            previous_stamp_ = target_stamp;
            return sent_;
        }
        UpdateTargetVelocity(wanted, target_stamp, dt);
        double remaining = std::min(dt, 0.02);
        while (remaining > 1e-12)
        {
            const double step = std::min(remaining, 0.005);
            const Eigen::Vector3d error = wanted.translation() - sent_.translation();
            velocity_ += Bound(Bound(follow_gain_ * error +
                feedforward_gain_ * target_velocity_, max_speed_) - velocity_, max_accel_ * step);
            sent_.translation() += velocity_ * step;
            Eigen::Quaterniond rotation_error(wanted.linear() * sent_.linear().transpose());
            rotation_error.normalize();
            if (rotation_error.w() < 0.0)
            {
                rotation_error.coeffs() *= -1.0;
            }
            const Eigen::AngleAxisd angle(rotation_error);
            const Eigen::Vector3d angular_error = angle.axis() * angle.angle();
            angular_velocity_ += Bound(Bound(8.0 * angular_error +
                feedforward_gain_ * target_angular_velocity_, 0.8) - angular_velocity_, 3.0 * step);
            const double speed = angular_velocity_.norm();
            if (speed > 1e-12)
            {
                sent_.linear() = Eigen::AngleAxisd(speed * step, angular_velocity_ / speed).toRotationMatrix() * sent_.linear();
            }
            remaining -= step;
        }
        return sent_;
    }
    const Eigen::Vector3d &Velocity() const { return velocity_; }
    const Eigen::Vector3d &AngularVelocity() const { return angular_velocity_; }

private:
    void UpdateTargetVelocity(const Eigen::Isometry3d &wanted, double stamp, double dt)
    {
        // Differentiate only new approved samples using their check-completion
        // timestamps. Repeated sends cannot manufacture a new input velocity.
        constexpr double filter_seconds = 0.04;
        const double sample_dt = stamp - previous_stamp_;
        if (feedforward_gain_ == 0.0 || stamp <= 0.0 || dt > 0.1 ||
            stamp < previous_stamp_ || sample_dt > 0.1)
        {
            target_velocity_.setZero();
            target_angular_velocity_.setZero();
        }
        else if (sample_dt >= 0.001 && previous_stamp_ > 0.0)
        {
            const double alpha = 1.0 - std::exp(-sample_dt / filter_seconds);
            target_velocity_ += alpha * (Bound(
                (wanted.translation() - previous_target_.translation()) / sample_dt,
                max_speed_) - target_velocity_);
            Eigen::Quaterniond delta(wanted.linear() * previous_target_.linear().transpose());
            delta.normalize();
            if (delta.w() < 0.0)
            {
                delta.coeffs() *= -1.0;
            }
            const Eigen::AngleAxisd angle(delta);
            target_angular_velocity_ += alpha * (Bound(
                angle.axis() * angle.angle() / sample_dt, 0.8) - target_angular_velocity_);
        }
        else
        {
            const double decay = std::exp(-dt / filter_seconds);
            target_velocity_ *= decay;
            target_angular_velocity_ *= decay;
        }
        if (stamp != previous_stamp_)
        {
            previous_target_ = wanted;
            previous_stamp_ = stamp;
        }
    }
    static Eigen::Vector3d Bound(const Eigen::Vector3d &value, double limit)
    {
        const double norm = value.norm();
        if (norm > limit)
        {
            return value * (limit / norm);
        }
        return value;
    }
    bool initialized_ = false;
    double max_speed_, max_accel_, follow_gain_;
    double feedforward_gain_;
    double previous_stamp_ = 0.0;
    Eigen::Isometry3d previous_target_ = Eigen::Isometry3d::Identity();
    Eigen::Vector3d target_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_angular_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Isometry3d sent_ = Eigen::Isometry3d::Identity();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
};
} // namespace smrcore::peripherals::detail
