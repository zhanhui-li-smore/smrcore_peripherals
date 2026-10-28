#pragma once
#include "fdcc_target_limiter.hpp"
#include "sdk/robot.hpp"
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <functional>
#include <utility>
#include <string>
#include <thread>

namespace smrcore::peripherals::detail
{
// One non-RT SDK worker, one latest-target slot. No queued trajectory and no
// ServoP/ServoJ fallback. Stop invalidates unsent targets; an in-flight SDK RPC
// remains subject to its own timeout and cannot be recalled by this adapter.
class PicoCompliance
{
public:
    using Clock = std::chrono::steady_clock;
    enum class Phase { Holding, Starting, Active, Stopping, Failed, Stopped };
    struct Status
    {
        Phase phase = Phase::Holding;
        std::string error;
        bool rearm_required = false;
        std::string pause_reason;
        uint64_t sent = 0;
        // SDK call duration; DDS target writes have no delivery acknowledgement.
        double call_ms = 0.0;
        // Same worker snapshot, all translation errors in base coordinates.
        double raw_error_mm = 0.0, lead_mm = 0.0;
        double shape_gap_mm = 0.0, follow_error_mm = 0.0;
        double shape_gap_deg = 0.0, follow_error_deg = 0.0;
        double target_speed_mps = 0.0, guard_age_ms = 0.0;
        double target_angular_speed_rps = 0.0;
        bool waiting_for_check = false;
    };

    struct Operations
    {
        const char *name;
        int active_mode;
        std::chrono::milliseconds period;
        std::function<void()> enable, manual_source, disable;
        std::function<void(const rcore::sdk::Pose &)> send;
        std::function<void(const rcore::RobotState &)> validate_feedback;
        std::function<void()> prepare_target;
    };

    PicoCompliance(rcore::sdk::Robot &robot, Operations operations,
                   double max_speed, double max_accel, double follow_gain,
                   double target_feedforward = 0.0)
        : robot_(robot), operations_(std::move(operations)),
          limiter_(max_speed, max_accel, follow_gain, target_feedforward), worker_([this] { Run(); }) {}
    virtual ~PicoCompliance() { Stop(); }
    PicoCompliance(const PicoCompliance &) = delete;
    PicoCompliance &operator=(const PicoCompliance &) = delete;

    Status Snapshot()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }
    // Updated only from fresh, tracked, held-Grip input, independently of the
    // IK/FK worker. Reusing a packet must reuse its original receive deadline.
    void UpdateInputDeadline(Clock::time_point deadline)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        input_deadline_ = deadline;
    }
    void Submit(const Eigen::Isometry3d &target, Clock::time_point valid_until)
    {
        Submit(target, valid_until, target);
    }
    void Submit(const Eigen::Isometry3d &target, Clock::time_point valid_until,
                const Eigen::Isometry3d &raw)
    {
        if (!target.matrix().allFinite() || !raw.matrix().allFinite())
        {
            throw Error("nonfinite target");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_ || status_.phase == Phase::Stopping || status_.rearm_required ||
            !status_.error.empty())
        {
            return;
        }
        if (Clock::now() > valid_until)
        {
            if (!input_deadline_)
            {
                PauseUntilReleaseLocked("checked target expired (>200ms)");
            }
            return;
        }
        if (!wanted_)
        {
            ++generation_;
            status_.phase = Phase::Starting;
        }
        wanted_ = true;
        target_ = target;
        raw_target_ = raw;
        valid_until_ = valid_until;
        wake_.notify_all();
    }
    void Pause()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (wanted_)
        {
            ++generation_;
            wanted_ = false;
            status_.phase = Phase::Stopping;
        }
        wake_.notify_all();
    }
    void PauseUntilRelease(const std::string &reason)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        PauseUntilReleaseLocked(reason);
    }
    // The caller must observe fresh, released Grip input. A late approval alone
    // cannot acknowledge this latch, nor can release skip the mode-0 exit.
    bool AcknowledgeGripRelease()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_ || wanted_ || status_.phase != Phase::Holding ||
            !status_.error.empty())
        {
            return false;
        }
        status_.rearm_required = false;
        status_.pause_reason.clear();
        return true;
    }
    void Stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
            wanted_ = false;
            ++generation_;
            wake_.notify_all();
        }
        if (worker_.joinable())
        {
            worker_.join();
        }
    }

protected:
    template <class Result> static void Check(Result result, const char *operation)
    {
        if (!result.IsSuccess())
        {
            throw std::runtime_error(std::string(operation) + ": [" +
                std::to_string(result.GetErrorCode()) + "] " + result.GetErrorMsg());
        }
    }
private:
    void PauseUntilReleaseLocked(const std::string &reason)
    {
        if (stop_ || !status_.error.empty())
        {
            return;
        }
        if (!status_.rearm_required)
        {
            status_.pause_reason = reason;
        }
        status_.rearm_required = true;
        wanted_ = false;
        ++generation_;
        status_.phase = Phase::Stopping;
        wake_.notify_all();
    }
    std::runtime_error Error(const std::string &message) const
    {
        return std::runtime_error(std::string(operations_.name) + ": " + message);
    }
    void Fail(const std::string &error)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.error += (status_.error.empty() ? "" : "; ") + error;
        status_.phase = Phase::Failed;
        wanted_ = false;
        ++generation_;
    }
    bool Current(uint64_t generation)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (wanted_ && Clock::now() > input_deadline_.value_or(valid_until_))
        {
            PauseUntilReleaseLocked(InputExpiryReasonLocked());
        }
        return !stop_ && wanted_ && generation_ == generation;
    }
    std::string InputExpiryReasonLocked() const
    {
        if (!input_deadline_)
        {
            return "checked target expired (>200ms)";
        }
        const double overdue_ms = std::chrono::duration<double, std::milli>(
            Clock::now() - *input_deadline_).count();
        return "Pico input deadline expired, overdue_ms=" + std::to_string(overdue_ms);
    }
    rcore::RobotState Feedback()
    {
        auto state = robot_.GetState();
        if (!robot_.IsConnected() || !state.cartesian_valid ||
            state.reference_frame != "base" || state.errors.GetLatchedErrorCount() > 0 ||
            !std::isfinite(state.timestamp) || state.timestamp <= 0.0)
        {
            throw Error("feedback disconnected/invalid/faulted or reference changed");
        }
        const auto now = Clock::now();
        if (state.timestamp != last_stamp_)
        {
            last_stamp_ = state.timestamp;
            last_feedback_ = now;
        }
        if (now - last_feedback_ > std::chrono::milliseconds(250))
        {
            throw Error("feedback stale (>250ms)");
        }
        if (operations_.validate_feedback)
        {
            operations_.validate_feedback(state);
        }
        return state;
    }
    Eigen::Isometry3d MeasuredPose(const rcore::RobotState &state)
    {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        for (int i = 0; i < 3; ++i)
        {
            if (!std::isfinite(state.cartesian_position[i]) ||
                !std::isfinite(state.cartesian_orientation[i]))
            {
                throw Error("TCP nonfinite");
            }
            pose.translation()[i] = state.cartesian_position[i];
        }
        pose.linear() = (Eigen::AngleAxisd(state.cartesian_orientation[2], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(state.cartesian_orientation[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(state.cartesian_orientation[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
        return pose;
    }
    bool WaitMode(int expected, uint64_t generation, bool cancellable)
    {
        const auto deadline = Clock::now() + std::chrono::seconds(1);
        while (Clock::now() < deadline)
        {
            if (cancellable && !Current(generation))
            {
                return false;
            }
            const auto state = Feedback();
            const int mode = static_cast<int>(state.control_mode);
            if (mode == expected)
            {
                return true;
            }
            if (mode != 0 && mode != operations_.active_mode)
            {
                throw Error("unexpected control mode during compliance transition");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        throw Error("mode feedback timeout");
    }
    void ExitMode()
    {
        if (mode_owned_)
        {
            operations_.disable();
            WaitMode(0, 0, false);
            mode_owned_ = false;
        }
        limiter_.Reset();
        last_sent_ = {};
        last_pose_.reset();
    }
    bool EnterMode(uint64_t generation)
    {
        if (!Current(generation))
        {
            return false;
        }
        if (static_cast<int>(Feedback().control_mode) != 0)
        {
            throw Error("enter compliance only from Kinematics (mode=0)");
        }
        // A timeout can mean activation succeeded: cleanup must already be armed.
        mode_owned_ = true;
        operations_.enable();
        if (!WaitMode(operations_.active_mode, generation, true))
        {
            return false;
        }
        operations_.manual_source();
        if (!Current(generation))
        {
            return false;
        }
        if (operations_.prepare_target)
        {
            operations_.prepare_target();
        }
        return Current(generation);
    }
    void Run()
    {
        try
        {
            for (;;)
            {
                Eigen::Isometry3d target, raw;
                Clock::time_point valid_until;
                uint64_t generation;
                bool wanted;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    wake_.wait_for(lock, std::chrono::milliseconds(2));
                    if (stop_)
                    {
                        break;
                    }
                    if (wanted_ && Clock::now() > input_deadline_.value_or(valid_until_))
                    {
                        PauseUntilReleaseLocked(InputExpiryReasonLocked());
                    }
                    wanted = wanted_;
                    target = target_;
                    raw = raw_target_;
                    valid_until = valid_until_;
                    generation = generation_;
                }
                if (!wanted)
                {
                    ExitMode();
                    std::lock_guard<std::mutex> lock(mutex_);
                    // Do not overwrite a newer Submit's Starting phase.
                    if (!wanted_)
                    {
                        status_.phase = Phase::Holding;
                        status_.waiting_for_check = false;
                    }
                    continue;
                }
                if (!mode_owned_)
                {
                    if (!EnterMode(generation))
                    {
                        ExitMode();
                    }
                    // Activation may take several RPCs. Consume a fresh slot
                    // on the next loop, not the target captured before Enable.
                    continue;
                }
                if (!Current(generation))
                {
                    ExitMode();
                    continue;
                }
                const auto state = Feedback();
                if (static_cast<int>(state.control_mode) != operations_.active_mode)
                {
                    throw Error("lost expected control mode; session stopped, no automatic re-entry");
                }
                const auto now = Clock::now();
                if (last_sent_ != Clock::time_point{} && now - last_sent_ < operations_.period)
                {
                    continue;
                }
                const double dt = last_sent_ == Clock::time_point{} ? 0.0 :
                    std::chrono::duration<double>(now - last_sent_).count();
                const auto measured = MeasuredPose(state);
                // A canceled/expired write must not advance the integrator.
                auto next_limiter = limiter_;
                const double target_stamp = std::chrono::duration<double>(
                    (valid_until - std::chrono::milliseconds(200)).time_since_epoch()).count();
                const bool waiting_for_check = now > valid_until;
                Eigen::Isometry3d sent;
                if (waiting_for_check)
                {
                    // Keep the last sent equilibrium point, without switching
                    // mode or advancing toward an old, now-stale candidate.
                    sent = last_pose_.value_or(measured);
                    next_limiter.Reset();
                    (void)next_limiter.Update(sent, sent, 0.0);
                }
                else
                {
                    sent = next_limiter.Update(target, measured, dt, target_stamp);
                }
                const Eigen::Vector3d rpy = sent.linear().eulerAngles(2, 1, 0).reverse();
                rcore::sdk::Pose pose;
                for (int i = 0; i < 3; ++i)
                {
                    pose.tvec()[i] = sent.translation()[i];
                    pose.rvec()[i] = rpy[i];
                }
                if (!Current(generation) || (!waiting_for_check && Clock::now() > valid_until))
                {
                    continue;
                }
                const auto started = Clock::now();
                operations_.send(pose);
                limiter_ = next_limiter;
                last_pose_ = sent;
                last_sent_ = started;
                const double call_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    ++status_.sent;
                    status_.call_ms = call_ms;
                    status_.raw_error_mm = 1000.0 * (raw.translation() - measured.translation()).norm();
                    status_.lead_mm = 1000.0 * (target.translation() - raw.translation()).norm();
                    status_.shape_gap_mm = 1000.0 * (target.translation() - sent.translation()).norm();
                    status_.follow_error_mm = 1000.0 * (sent.translation() - measured.translation()).norm();
                    constexpr double radians_to_degrees = 57.29577951308232;
                    status_.shape_gap_deg = radians_to_degrees * Eigen::Quaterniond(
                        target.linear()).angularDistance(Eigen::Quaterniond(sent.linear()));
                    status_.follow_error_deg = radians_to_degrees * Eigen::Quaterniond(
                        sent.linear()).angularDistance(Eigen::Quaterniond(measured.linear()));
                    status_.target_speed_mps = limiter_.Velocity().norm();
                    status_.target_angular_speed_rps = limiter_.AngularVelocity().norm();
                    status_.guard_age_ms = 200.0 + std::chrono::duration<double, std::milli>(started - valid_until).count();
                    status_.waiting_for_check = waiting_for_check;
                    if (wanted_ && generation_ == generation)
                    {
                        status_.phase = Phase::Active;
                    }
                }
                if (call_ms > 200.0)
                {
                    throw Error("target write exceeded 200ms; target freshness lost");
                }
            }
        }
        catch (const std::exception &error)
        {
            Fail(error.what());
        }
        catch (...)
        {
            Fail("worker unknown exception");
        }
        try
        {
            ExitMode();
        }
        catch (const std::exception &error)
        {
            Fail(error.what());
            try { robot_.Motion().StopMotion(); } catch (...) {}
            try { robot_.Disable(); } catch (...) {}
        }
        catch (...)
        {
            Fail("cleanup unknown exception");
            try { robot_.Motion().StopMotion(); } catch (...) {}
            try { robot_.Disable(); } catch (...) {}
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.error.empty())
        {
            status_.phase = Phase::Stopped;
        }
    }
    rcore::sdk::Robot &robot_;
    const Operations operations_;
    std::mutex mutex_;
    std::condition_variable wake_;
    Status status_;
    bool stop_ = false, wanted_ = false;
    uint64_t generation_ = 0;
    Eigen::Isometry3d target_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d raw_target_ = Eigen::Isometry3d::Identity();
    Clock::time_point valid_until_{};
    std::optional<Clock::time_point> input_deadline_;
    // Worker-owned state below.
    bool mode_owned_ = false;
    double last_stamp_ = 0.0;
    Clock::time_point last_feedback_{}, last_sent_{};
    std::optional<Eigen::Isometry3d> last_pose_;
    FdccTargetLimiter limiter_;
    std::thread worker_;
};
} // namespace smrcore::peripherals::detail
