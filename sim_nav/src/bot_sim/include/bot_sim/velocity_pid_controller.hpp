/**
 * @file velocity_pid_controller.hpp
 * @brief Header-only PID velocity controller that sits between the D*-Lite
 *        path planner and the /cmd_vel publisher.
 *
 * DESIGN OVERVIEW
 * ---------------
 * The D*-Lite planner computes a "desired" body-frame velocity each control
 * cycle.  Without feedback, actuator lag and localization drift cause the
 * robot to over- or under-shoot the intended velocity.  This controller
 * closes the loop:
 *
 *   desired_cmd  ──►  VelocityPidController::update()  ──►  corrected /cmd_vel
 *
 * Feedback is obtained by differentiating the robot's TF position (map →
 * robot_frame) rather than requiring a dedicated odometry topic, which makes
 * the controller work with any localization backend (LOAM, HDL, etc.).
 *
 * CONTROL LAW (per linear axis)
 * ------------------------------
 *   u = ff * desired  +  Kp * e  +  Ki * ∫e dt  +  Kd * de/dt
 *
 * where:
 *   ff      = feedforward gain (default 1.0, makes desired the nominal output)
 *   e       = desired − measured  (zeroed inside deadband for P and I)
 *   ∫e dt   = clamped to ±integral_limit to prevent windup
 *   de/dt   = derivative of raw_error (un-zeroed); suppressed when both the
 *             current and previous errors are inside the deadband to avoid
 *             amplifying measurement noise near the setpoint
 *
 * After per-axis computation the output vector is magnitude-clamped to
 * output_limit and then rate-limited by accel_limit (m/s²).  If feedback is
 * temporarily unavailable, the controller falls back to the planner command
 * but still applies the output magnitude clamp before publishing.
 *
 * TYPICAL USAGE
 * -------------
 * @code
 *   VelocityPidController pid;
 *   pid.loadParams(private_nh, max_speed);   // once, during node init
 *   ...
 *   // inside the planning loop:
 *   if (pid.enabled()) {
 *       cmd_vel = pid.update(raw_cmd, "map", "virtual_frame", tf_buffer_);
 *   } else {
 *       cmd_vel = raw_cmd;
 *   }
 * @endcode
 *
 * THREAD SAFETY
 * -------------
 * Not thread-safe.  All calls must originate from the same thread (the
 * planning loop).  The shared tf2_ros::Buffer is internally locked and may
 * safely be accessed from multiple threads.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

namespace bot_sim {

/**
 * @class VelocityPidController
 * @brief Feedforward + PID body-frame velocity controller with TF feedback.
 *
 * Wraps the D*-Lite planner's raw velocity command and corrects it using
 * feedback derived from the map → robot_frame TF transform.  Angular velocity
 * (yaw rate) is always passed through unmodified because heading alignment is
 * handled separately by the planner's own logic.
 *
 * The controller can be disabled entirely at runtime via the ROS parameter
 * `pid_enabled` (default: false).  When disabled, update() is a zero-cost
 * passthrough with no state modifications.
 */
class VelocityPidController {
public:
    /**
     * @brief Construct with safe conservative defaults.
     *
     * All tuning values default to conservative, stable settings.  Call
     * loadParams() after construction to override from the ROS parameter
     * server.
     */
    VelocityPidController()
        : enabled_(false), feedforward_(1.0), integral_limit_(0.5),
          output_limit_(1.5), accel_limit_(2.0), deadband_(0.03),
          feedback_timeout_(0.25), reset_on_stop_(true), stop_epsilon_(1e-3),
          has_pose_(false), has_output_(false) {}

    /**
     * @brief Load all PID tuning parameters from the ROS parameter server.
     *
     * Must be called once during node initialisation, typically inside the
     * node constructor after the private NodeHandle is created.  Each
     * parameter is optional; the current member value is used as the default
     * when the parameter is absent.
     *
     * @param private_nh           Private NodeHandle ("~") so parameter names
     *                             are scoped to the node's own namespace.
     * @param default_output_limit Fallback for `pid_output_limit`.  Pass the
     *                             planner's maximum speed so the PID output
     *                             never exceeds the physical limit even when
     *                             the parameter is not explicitly set.
     *
     * ROS Parameters (all optional — defaults in parentheses):
     *
     *  pid_enabled         (false)  Master switch.  true = active control;
     *                               false = passthrough (zero CPU overhead).
     *
     *  pid_feedforward     (1.0)    Gain on the desired velocity before
     *                               adding PID correction.  At 1.0 the
     *                               planner command is the nominal output and
     *                               PID only corrects residual error.
     *                               Lowering this shifts more authority to
     *                               the integral term.
     *
     *  pid_kp_x / pid_kp_y (0.35)  Proportional gains for the forward (X)
     *                               and lateral (Y) body-frame axes
     *                               [m/s per m/s error].  Increase to react
     *                               faster; too high causes oscillation.
     *
     *  pid_ki_x / pid_ki_y (0.0)   Integral gains [m/s per (m/s)·s].
     *                               Keep at 0 unless steady-state velocity
     *                               error is observed; non-zero values must
     *                               be paired with a sensible integral_limit.
     *
     *  pid_kd_x / pid_kd_y (0.02)  Derivative gains [m/s per (m/s)/s].
     *                               Damps oscillation from actuation lag.
     *                               Large values amplify TF position noise.
     *
     *  pid_integral_limit  (0.5)   Symmetric clamp on each axis's integrator
     *                               [m/s].  Prevents windup when the robot is
     *                               blocked, saturated, or tracking a large
     *                               sustained error.
     *
     *  pid_output_limit    (1.5)   Maximum output velocity vector magnitude
     *                               [m/s].  Both axes are scaled together to
     *                               preserve the commanded direction.
     *
     *  pid_accel_limit     (2.0)   Maximum rate of change of the output
     *                               velocity vector between consecutive cycles
     *                               [m/s²].  Prevents instantaneous jumps
     *                               after a stop, goal change, or TF hiccup.
     *
     *  pid_deadband        (0.03)  Velocity error magnitude below which the
     *                               P and I terms are zeroed [m/s].  Reduces
     *                               chatter near the setpoint.  The D term
     *                               uses the raw (un-zeroed) error to avoid
     *                               a discontinuous spike at the boundary.
     *
     *  pid_feedback_timeout (0.25) Maximum acceptable gap between two
     *                               consecutive TF position samples [s].
     *                               If dt exceeds this the velocity estimate
     *                               is considered stale and the controller
     *                               resets its derivative/integral state.
     *
     *  pid_reset_on_stop   (true)  When true, a near-zero desired speed
     *                               (|v| < stop_epsilon_) immediately clears
     *                               the integrators and returns a zero
     *                               command, preventing an integral lurch on
     *                               the next motion segment.
     */
    void loadParams(const ros::NodeHandle& private_nh, double default_output_limit) {
        private_nh.param("pid_enabled", enabled_, false);
        private_nh.param("pid_feedforward", feedforward_, feedforward_);
        private_nh.param("pid_integral_limit", integral_limit_, integral_limit_);
        private_nh.param("pid_output_limit", output_limit_, default_output_limit);
        private_nh.param("pid_accel_limit", accel_limit_, accel_limit_);
        private_nh.param("pid_deadband", deadband_, deadband_);
        private_nh.param("pid_feedback_timeout", feedback_timeout_, feedback_timeout_);
        private_nh.param("pid_reset_on_stop", reset_on_stop_, reset_on_stop_);

        private_nh.param("pid_kp_x", x_axis_.kp, 0.35);
        private_nh.param("pid_ki_x", x_axis_.ki, 0.0);
        private_nh.param("pid_kd_x", x_axis_.kd, 0.02);
        private_nh.param("pid_kp_y", y_axis_.kp, 0.35);
        private_nh.param("pid_ki_y", y_axis_.ki, 0.0);
        private_nh.param("pid_kd_y", y_axis_.kd, 0.02);

        ROS_INFO("Velocity PID params: enabled=%s ff=%.3f kp=(%.3f, %.3f) ki=(%.3f, %.3f) kd=(%.3f, %.3f) output_limit=%.3f accel_limit=%.3f deadband=%.3f feedback_timeout=%.3f",
             enabled_ ? "true" : "false",
             feedforward_,
             x_axis_.kp, y_axis_.kp,
             x_axis_.ki, y_axis_.ki,
             x_axis_.kd, y_axis_.kd,
             output_limit_, accel_limit_, deadband_, feedback_timeout_);
    }

    /**
     * @brief Return whether the controller is active.
     *
     * Callers may use this to skip unrelated work (e.g. publishing a
     * raw_cmd_vel diagnostic) when the controller is in passthrough mode.
     */
    bool enabled() const { return enabled_; }

    /**
     * @brief Reset all dynamic state to initial conditions.
     *
     * Clears:
     *  - Both axis integrators and derivative history
     *  - Stored TF pose sample (has_pose_, last_pose_*)
     *  - Stored last output for the acceleration limiter (has_output_, last_output_)
     *
     * Call this whenever accumulated state would produce a meaningless or
     * dangerous correction on the next cycle, e.g.:
     *  - Goal change (a new trajectory begins)
     *  - Robot commanded to stop (optionally automated via reset_on_stop_)
     *  - TF feedback becomes unavailable
     */
    void reset() {
        x_axis_.reset();
        y_axis_.reset();
        has_pose_ = false;
        has_output_ = false;
        last_output_ = geometry_msgs::Twist();
    }

    /**
     * @brief Compute and return the PID-corrected velocity command.
     *
     * Main entry point, called once per planning cycle.  Executes the full
     * control loop in the following order:
     *
     *  1. **Passthrough** — if !enabled_, return @p desired unchanged (no state
     *     modification).
        *  2. **Stop-reset** — if desired speed ≈ 0 and reset_on_stop_ is set,
        *     reset state and return a zero command to prevent integral lurch.
        *  3. **Feedback estimation** — call estimateMeasuredVelocity() to derive
        *     body-frame velocity from TF position differentiation.  On a repeated
        *     TF timestamp, hold the last corrected output instead of falling back
        *     to raw.  On TF failure or first-sample initialization, the desired
        *     command is used as a fallback, but it still passes through the output
        *     magnitude clamp.
     *  4. **Per-axis PID** — updateAxis() for X and Y independently.
     *  5. **Output magnitude clamp** — the (x, y) velocity vector is scaled
     *     down to output_limit_ while preserving its direction.
     *  6. **Acceleration rate limit** — the change from the previous output
     *     is clamped to accel_limit_ × dt, also preserving direction.
     *
     * @param desired          Raw velocity command from the planner
     *                         (body frame, m/s and rad/s).
     * @param map_frame_name   Fixed planning frame (e.g. "map").
     * @param robot_frame_name Robot body frame (e.g. "virtual_frame").
     * @param tf_buffer        Shared TF buffer; must have the
     *                         map → robot_frame transform populated.
     * @return Corrected velocity command in the same convention as @p desired.
     *         Angular Z is forwarded without modification.
     */
    geometry_msgs::Twist update(const geometry_msgs::Twist& desired,
                                const std::string& map_frame_name,
                                const std::string& robot_frame_name,
                                tf2_ros::Buffer& tf_buffer) {
        // ── 1. Master switch ──────────────────────────────────────────────
        if (!enabled_) {
            return desired;
        }

        // ── 2. Stop-reset ─────────────────────────────────────────────────
        // When the planner issues a near-zero velocity, clear accumulated
        // state so the integrator does not cause a lurch on the next segment.
        if (reset_on_stop_ && std::hypot(desired.linear.x, desired.linear.y) < stop_epsilon_) {
            reset();
            return geometry_msgs::Twist();
        }

        // ── 3. Measure body-frame velocity from TF position differentiation ──
        double measured_x = 0.0;
        double measured_y = 0.0;
        double dt = 0.0;
        const FeedbackStatus feedback_status = estimateMeasuredVelocity(
            map_frame_name, robot_frame_name, tf_buffer, measured_x, measured_y, dt);

        if (feedback_status == FeedbackStatus::NoNewSample) {
            return holdPreviousOutput(desired);
        }

        if (feedback_status != FeedbackStatus::Valid) {
            // TF unavailable, stale, or first sample: do not apply PID terms,
            // but still enforce the final velocity cap.  This prevents
            // /cmd_vel from becoming identical to an over-limit raw_cmd_vel
            // during fallback cycles.
            return limitedFallback(desired);
        }

        // ── 4. Per-axis PID correction ────────────────────────────────────
        geometry_msgs::Twist output = desired;
        output.linear.x = updateAxis(x_axis_, desired.linear.x, measured_x, dt);
        output.linear.y = updateAxis(y_axis_, desired.linear.y, measured_y, dt);
        output.angular.z = desired.angular.z;  // yaw is managed by the planner

        // ── 5. Clamp output vector magnitude ─────────────────────────────
        limitVector(output.linear.x, output.linear.y, output_limit_);

        // ── 6. Rate-limit velocity changes ───────────────────────────────
        limitAcceleration(output, dt);
        return output;
    }

private:
    enum class FeedbackStatus {
        Valid,
        NoNewSample,
        Unavailable
    };

    /**
     * @struct AxisState
     * @brief Per-axis PID gains and dynamic state.
     *
     * One instance is kept for each controlled axis (X forward, Y lateral).
     * Gains are loaded from the parameter server at startup and never change
     * at runtime.  Dynamic state (integral, derivative history) is cleared
     * by reset().
     */
    struct AxisState {
        double kp = 0.0;  ///< Proportional gain  [m/s per m/s error]
        double ki = 0.0;  ///< Integral gain       [m/s per (m/s)·s]
        double kd = 0.0;  ///< Derivative gain     [m/s per (m/s)/s]

        /// Accumulated integral term; clamped to ±integral_limit_ to prevent windup.
        double integral = 0.0;

        /// Error value from the previous cycle; used to compute de/dt.
        /// Stores raw_error (not the deadband-zeroed version) for smooth
        /// derivative across the deadband boundary.
        double previous_error = 0.0;

        /// False on the first cycle; the derivative term is skipped until
        /// a valid previous_error is available.
        bool has_previous_error = false;

        /** @brief Clear dynamic state; gains are preserved. */
        void reset() {
            integral = 0.0;
            previous_error = 0.0;
            has_previous_error = false;
        }
    };

    /**
     * @brief Symmetric scalar clamp: result ∈ [−limit, +limit].
     *
     * A non-positive limit is treated as "unlimited" — the value is returned
     * unchanged.  This allows any limit to be disabled at runtime by setting
     * the corresponding parameter to 0 or negative.
     */
    static double clamp(double value, double limit) {
        if (limit <= 0.0) {
            return value;  // disabled
        }
        return std::max(-limit, std::min(limit, value));
    }

    /**
     * @brief Scale a 2-D vector so its Euclidean magnitude does not exceed
     *        @p limit, preserving the original direction.
     *
     * Both components are multiplied by the same scale factor, so the
     * commanded direction of motion is unchanged after clamping.  A
     * non-positive limit is a no-op.
     *
     * @param[in,out] x  X component.
     * @param[in,out] y  Y component.
     * @param limit      Maximum allowable vector magnitude.
     */
    static void limitVector(double& x, double& y, double limit) {
        if (limit <= 0.0) {
            return;  // disabled
        }
        const double norm = std::hypot(x, y);
        if (norm > limit) {
            const double scale = limit / norm;
            x *= scale;
            y *= scale;
        }
    }

    /**
     * @brief Execute one PID update step for a single axis.
     *
     * Control law in detail:
     * @code
     *   raw_error     = desired − measured
     *
     *   // Deadband: zero P and I inside the noise floor to stop chatter.
     *   control_error = (|raw_error| < deadband) ? 0.0 : raw_error
     *
     *   // Integral with anti-windup symmetric clamp.
     *   integral += control_error * dt           (clamped to ±integral_limit)
     *
     *   // Derivative on raw_error to avoid a step-discontinuity at the
     *   // deadband boundary.  Suppressed entirely when both current and
     *   // previous errors are inside the deadband (noise dominates there).
     *   derivative = (raw_error − previous_raw_error) / dt
     *   if |raw_error| < deadband && |prev_error| < deadband:
     *       derivative = 0
     *
     *   u = ff * desired + Kp * control_error + Ki * integral + Kd * derivative
     * @endcode
     *
     * @param axis     Mutable axis state (gains + integrator + error history).
     * @param desired  Desired velocity for this axis [m/s].
     * @param measured Measured body-frame velocity for this axis [m/s].
     * @param dt       Time elapsed since the previous update [s].
     * @return Corrected velocity command for this axis [m/s].
     */
    double updateAxis(AxisState& axis, double desired, double measured, double dt) {
        // Compute the raw tracking error (full precision, not deadband-zeroed).
        const double raw_error = desired - measured;

        // Apply deadband to P and I: prevents fighting measurement noise when
        // the robot is already near the velocity setpoint.
        // The derivative continues to use raw_error (see below).
        const double control_error = std::abs(raw_error) < deadband_ ? 0.0 : raw_error;

        // Integrate control_error (deadband-zeroed) with anti-windup clamp.
        axis.integral = clamp(axis.integral + control_error * dt, integral_limit_);

        // Derivative term — only valid from the second cycle onward.
        double derivative = 0.0;
        if (axis.has_previous_error) {
            // Use raw_error (not control_error) so the derivative is continuous
            // when the error crosses the deadband boundary.
            derivative = (raw_error - axis.previous_error) / dt;

            // Both current and previous errors inside the deadband means the
            // robot is already well-controlled and the position noise floor
            // dominates.  Zero the derivative to avoid amplifying that noise.
            if (std::abs(raw_error) < deadband_ && std::abs(axis.previous_error) < deadband_) {
                derivative = 0.0;
            }
        }

        // Store raw_error so the next cycle's derivative is smooth across the
        // deadband boundary.
        axis.previous_error = raw_error;
        axis.has_previous_error = true;

        // Feedforward baseline (planner command) + PID correction.
        return feedforward_ * desired
             + axis.kp * control_error
             + axis.ki * axis.integral
             + axis.kd * derivative;
    }

    /**
     * @brief Estimate the robot's body-frame velocity by differentiating the
     *        TF position between consecutive planning cycles.
     *
     * APPROACH
     * --------
     *  1. Look up map → robot_frame (position) and robot_frame → map (rotation
     *     only, used to rotate the velocity into body frame).
     *  2. Compute the map-frame displacement since the previous sample:
     *       v_map = (pos_now − pos_prev) / dt
     *  3. Rotate v_map into the robot body frame using the pure-rotation
     *     transform (translation components zeroed to avoid a frame-origin
     *     offset in the rotated velocity).
     *  4. Return the body-frame velocity components.
     *
    * On the very first call (has_pose_ == false) the sample is stored and
    * FeedbackStatus::Unavailable is returned — the caller must wait for the
    * second call to obtain a valid velocity estimate.  When the planner loop
    * runs faster than TF publication and the timestamp has not advanced,
    * FeedbackStatus::NoNewSample is returned so the caller can keep publishing
    * the previous corrected output instead of erasing the PID correction.
     *
     * @param map_frame_name    Fixed planning frame (e.g. "map").
     * @param robot_frame_name  Body frame (e.g. "virtual_frame").
     * @param tf_buffer         Shared TF buffer.
     * @param[out] measured_x   Body-frame X velocity [m/s].
     * @param[out] measured_y   Body-frame Y velocity [m/s].
     * @param[out] dt           Time between the last two TF samples [s].
    * @return FeedbackStatus::Valid when measured_x/y and dt are valid.
    * @return FeedbackStatus::NoNewSample when TF is present but its timestamp
    *         has not advanced since the previous controller cycle.
    * @return FeedbackStatus::Unavailable if TF is missing, this is the first
    *         sample, or the TF gap is stale (> feedback_timeout_).
     */
    FeedbackStatus estimateMeasuredVelocity(const std::string& map_frame_name,
                                    const std::string& robot_frame_name,
                                    tf2_ros::Buffer& tf_buffer,
                                    double& measured_x,
                                    double& measured_y,
                                    double& dt) {
        geometry_msgs::TransformStamped pose_transform;
        geometry_msgs::TransformStamped robot_from_map_transform;
        try {
            // ros::Time(0) means "most recent available transform".
            pose_transform = tf_buffer.lookupTransform(
                map_frame_name, robot_frame_name, ros::Time(0));
            // Inverse direction: used to rotate the map-frame velocity vector
            // into the robot body frame.
            robot_from_map_transform = tf_buffer.lookupTransform(
                robot_frame_name, map_frame_name, ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "Velocity PID feedback TF unavailable: %s", ex.what());
            reset();  // clear stale state; next cycle restarts from scratch
            return FeedbackStatus::Unavailable;
        }

        // Use the transform's own timestamp when available; fall back to
        // ros::Time::now() for TF trees that publish with zero stamps.
        const ros::Time stamp = pose_transform.header.stamp.isZero()
                                    ? ros::Time::now()
                                    : pose_transform.header.stamp;
        const double x = pose_transform.transform.translation.x;
        const double y = pose_transform.transform.translation.y;

        // First call: record the initial position and return false.
        // A velocity estimate requires two samples.
        if (!has_pose_) {
            last_pose_stamp_ = stamp;
            last_pose_x_ = x;
            last_pose_y_ = y;
            has_pose_ = true;
            return FeedbackStatus::Unavailable;
        }

        // Compute elapsed time.
        dt = (stamp - last_pose_stamp_).toSec();

        // TF updates at 25 Hz but the planner loop runs at ~100 Hz.  When the
        // TF timestamp has not advanced (dt ≈ 0), the transform is simply a
        // repeat of the previous one.  Tell the caller to hold the last
        // corrected output instead of falling back to the raw planner command.
        if (dt <= 1e-4) {
            return FeedbackStatus::NoNewSample;
        }

        // Stale transform: the gap is too large for a reliable velocity
        // estimate.  Reset axis state so the next valid sample restarts from
        // a clean slate rather than using a derivative spanning a multi-second
        // gap.
        if (dt > feedback_timeout_) {
            x_axis_.reset();
            y_axis_.reset();
            // Advance the sliding window so the next cycle measures from now.
            last_pose_stamp_ = stamp;
            last_pose_x_ = x;
            last_pose_y_ = y;
            return FeedbackStatus::Unavailable;
        }

        // Valid new sample — advance the sliding window.
        last_pose_stamp_ = stamp;
        const double previous_x = last_pose_x_;
        const double previous_y = last_pose_y_;
        last_pose_x_ = x;
        last_pose_y_ = y;

        // Finite-difference velocity in the fixed map frame.
        const double vx_map = (x - previous_x) / dt;
        const double vy_map = (y - previous_y) / dt;

        // Zero the translation part of the robot_from_map transform so that
        // tf2::Transform applies a pure rotation without a spurious offset
        // from the frame origin.
        robot_from_map_transform.transform.translation.x = 0.0;
        robot_from_map_transform.transform.translation.y = 0.0;
        robot_from_map_transform.transform.translation.z = 0.0;

        // Rotate map-frame velocity into the robot body frame.
        tf2::Transform robot_from_map;
        tf2::fromMsg(robot_from_map_transform.transform, robot_from_map);
        const tf2::Vector3 measured_robot =
            robot_from_map * tf2::Vector3(vx_map, vy_map, 0.0);
        measured_x = measured_robot.x();
        measured_y = measured_robot.y();
        return FeedbackStatus::Valid;
    }

    /**
     * @brief Rate-limit the output velocity to at most accel_limit_ × dt
     *        per planning cycle.
     *
     * The (Δx, Δy) change from the previous output is computed, its
     * magnitude is clamped with limitVector(), and the output is rebuilt as
     * last_output_ + clamped_delta — preserving the direction of change.
     *
     * Special bypass cases (rememberOutput() is still called in both):
     *  - accel_limit_ ≤ 0 or dt ≤ 0: acceleration limiting is disabled;
     *    output passes through unchanged.
     *  - First cycle (has_output_ == false): no previous output exists so
     *    no ramp can be applied.  The current output is accepted as-is and
     *    stored to seed the limiter for the next cycle.
     *
     * @param[in,out] output  Velocity command to rate-limit in place.
     * @param dt              Time elapsed since the previous update [s].
     */
    void limitAcceleration(geometry_msgs::Twist& output, double dt) {
        if (accel_limit_ <= 0.0 || dt <= 0.0) {
            // Limiting disabled; store output for next cycle's baseline.
            rememberOutput(output);
            return;
        }

        // First cycle: no previous output to ramp from.  Accept the current
        // value and seed last_output_ so subsequent cycles have a valid base.
        if (!has_output_) {
            rememberOutput(output);
            return;
        }

        // Compute the requested velocity change vector.
        double delta_x = output.linear.x - last_output_.linear.x;
        double delta_y = output.linear.y - last_output_.linear.y;

        // Clamp the change magnitude to accel_limit_ * dt [m/s] while
        // preserving the direction of the change vector.
        limitVector(delta_x, delta_y, accel_limit_ * dt);

        // Rebuild the output from the stored base plus the clamped delta.
        output.linear.x = last_output_.linear.x + delta_x;
        output.linear.y = last_output_.linear.y + delta_y;
        last_output_ = output;  // advance the sliding window
    }

    /**
     * @brief Record @p output as the last issued command.
     *
     * Must be called for EVERY command returned by update(), including
     * passthrough and TF-failure cases, so that limitAcceleration() always
     * has a valid previous output to ramp from.  Omitting this call after a
     * passthrough would cause a step-jump in the acceleration limiter on the
     * first "active" PID cycle.
     *
     * @param output  The command that is about to be (or has been) published.
     */
    void rememberOutput(const geometry_msgs::Twist& output) {
        last_output_ = output;
        has_output_ = true;
    }

    /**
     * @brief Fallback command path used when PID feedback is not valid.
     *
     * This intentionally does not apply P/I/D terms because measured velocity
     * is unavailable or stale.  It does, however, apply output_limit_ so the
     * final /cmd_vel cannot exceed the configured safety cap while the raw
     * planner command remains visible on raw_cmd_vel for debugging.
     */
    geometry_msgs::Twist limitedFallback(const geometry_msgs::Twist& desired) {
        geometry_msgs::Twist output = desired;
        limitVector(output.linear.x, output.linear.y, output_limit_);
        rememberOutput(output);
        ROS_DEBUG_THROTTLE(1.0,
                           "Velocity PID fallback: publishing magnitude-limited raw command");
        return output;
    }

    /**
     * @brief Keep publishing the last corrected output between TF updates.
     *
     * The planner loop can run several times for every new TF sample.  During
     * those repeated-timestamp cycles, publishing raw desired velocity would
     * erase the correction produced on the previous valid feedback sample and
     * make /cmd_vel look identical to raw_cmd_vel even while the robot is not
     * tracking the command.  Holding last_output_ preserves the correction
     * until a fresh measured velocity arrives.
     */
    geometry_msgs::Twist holdPreviousOutput(const geometry_msgs::Twist& desired) {
        if (!has_output_) {
            return limitedFallback(desired);
        }

        geometry_msgs::Twist output = last_output_;
        output.angular.z = desired.angular.z;
        limitVector(output.linear.x, output.linear.y, output_limit_);
        rememberOutput(output);
        ROS_DEBUG_THROTTLE(1.0,
                           "Velocity PID hold: reusing previous corrected command until TF advances");
        return output;
    }

    // ── Axis state ──────────────────────────────────────────────────────────
    AxisState x_axis_;  ///< PID state for the forward  (X) body-frame axis
    AxisState y_axis_;  ///< PID state for the lateral  (Y) body-frame axis

    // ── Scalar tuning parameters (loaded from ROS param server) ────────────
    bool   enabled_;           ///< Master enable flag            (pid_enabled)
    double feedforward_;       ///< Feedforward gain on desired   (pid_feedforward)
    double integral_limit_;    ///< Symmetric integrator clamp [m/s] (pid_integral_limit)
    double output_limit_;      ///< Max output vector magnitude [m/s] (pid_output_limit)
    double accel_limit_;       ///< Max output rate of change [m/s²] (pid_accel_limit)
    double deadband_;          ///< Error deadband for P/I [m/s]  (pid_deadband)
    double feedback_timeout_;  ///< Max TF sample gap [s]         (pid_feedback_timeout)
    bool   reset_on_stop_;     ///< Clear state on near-zero cmd  (pid_reset_on_stop)

    /// Speed magnitude below which the robot is considered stopped [m/s].
    /// Hard-coded; not exposed as a parameter (intentionally tight).
    double stop_epsilon_;

    // ── TF feedback state ───────────────────────────────────────────────────
    bool      has_pose_;        ///< True after the first successful TF lookup
    ros::Time last_pose_stamp_; ///< Timestamp of the most recent TF sample
    double    last_pose_x_ = 0.0; ///< Map-frame X of the most recent TF sample [m]
    double    last_pose_y_ = 0.0; ///< Map-frame Y of the most recent TF sample [m]

    // ── Acceleration limiter state ──────────────────────────────────────────
    bool                 has_output_;  ///< True after rememberOutput() is called at least once
    geometry_msgs::Twist last_output_; ///< Last command issued; base for the rate-limit ramp
};

}  // namespace bot_sim