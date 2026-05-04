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

class VelocityPidController {
public:
    VelocityPidController()
        : enabled_(false), feedforward_(1.0), integral_limit_(0.5),
          output_limit_(1.5), accel_limit_(2.0), deadband_(0.03),
          feedback_timeout_(0.25), reset_on_stop_(true), stop_epsilon_(1e-3),
          has_pose_(false), has_output_(false) {}

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
    }

    bool enabled() const { return enabled_; }

    void reset() {
        x_axis_.reset();
        y_axis_.reset();
        has_pose_ = false;
        has_output_ = false;
        last_output_ = geometry_msgs::Twist();
    }

    geometry_msgs::Twist update(const geometry_msgs::Twist& desired,
                                const std::string& map_frame_name,
                                const std::string& robot_frame_name,
                                tf2_ros::Buffer& tf_buffer) {
        if (!enabled_) {
            return desired;
        }

        if (reset_on_stop_ && std::hypot(desired.linear.x, desired.linear.y) < stop_epsilon_) {
            reset();
            return geometry_msgs::Twist();
        }

        double measured_x = 0.0;
        double measured_y = 0.0;
        double dt = 0.0;
        if (!estimateMeasuredVelocity(map_frame_name, robot_frame_name, tf_buffer, measured_x, measured_y, dt)) {
            rememberOutput(desired);
            return desired;
        }

        geometry_msgs::Twist output = desired;
        output.linear.x = updateAxis(x_axis_, desired.linear.x, measured_x, dt);
        output.linear.y = updateAxis(y_axis_, desired.linear.y, measured_y, dt);
        output.angular.z = desired.angular.z;

        limitVector(output.linear.x, output.linear.y, output_limit_);
        limitAcceleration(output, dt);
        return output;
    }

private:
    struct AxisState {
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
        double integral = 0.0;
        double previous_error = 0.0;
        bool has_previous_error = false;

        void reset() {
            integral = 0.0;
            previous_error = 0.0;
            has_previous_error = false;
        }
    };

    static double clamp(double value, double limit) {
        if (limit <= 0.0) {
            return value;
        }
        return std::max(-limit, std::min(limit, value));
    }

    static void limitVector(double& x, double& y, double limit) {
        if (limit <= 0.0) {
            return;
        }
        const double norm = std::hypot(x, y);
        if (norm > limit) {
            const double scale = limit / norm;
            x *= scale;
            y *= scale;
        }
    }

    double updateAxis(AxisState& axis, double desired, double measured, double dt) {
        const double raw_error = desired - measured;
        const double control_error = std::abs(raw_error) < deadband_ ? 0.0 : raw_error;

        axis.integral = clamp(axis.integral + control_error * dt, integral_limit_);

        double derivative = 0.0;
        if (axis.has_previous_error) {
            derivative = (raw_error - axis.previous_error) / dt;
            if (std::abs(raw_error) < deadband_ && std::abs(axis.previous_error) < deadband_) {
                derivative = 0.0;
            }
        }

        axis.previous_error = raw_error;
        axis.has_previous_error = true;

        return feedforward_ * desired + axis.kp * control_error + axis.ki * axis.integral + axis.kd * derivative;
    }

    bool estimateMeasuredVelocity(const std::string& map_frame_name,
                                  const std::string& robot_frame_name,
                                  tf2_ros::Buffer& tf_buffer,
                                  double& measured_x,
                                  double& measured_y,
                                  double& dt) {
        geometry_msgs::TransformStamped pose_transform;
        geometry_msgs::TransformStamped robot_from_map_transform;
        try {
            pose_transform = tf_buffer.lookupTransform(map_frame_name, robot_frame_name, ros::Time(0));
            robot_from_map_transform = tf_buffer.lookupTransform(robot_frame_name, map_frame_name, ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "Velocity PID feedback TF unavailable: %s", ex.what());
            reset();
            return false;
        }

        const ros::Time stamp = pose_transform.header.stamp.isZero() ? ros::Time::now() : pose_transform.header.stamp;
        const double x = pose_transform.transform.translation.x;
        const double y = pose_transform.transform.translation.y;

        if (!has_pose_) {
            last_pose_stamp_ = stamp;
            last_pose_x_ = x;
            last_pose_y_ = y;
            has_pose_ = true;
            return false;
        }

        dt = (stamp - last_pose_stamp_).toSec();
        last_pose_stamp_ = stamp;
        const double previous_x = last_pose_x_;
        const double previous_y = last_pose_y_;
        last_pose_x_ = x;
        last_pose_y_ = y;

        if (dt <= 1e-4 || dt > feedback_timeout_) {
            x_axis_.reset();
            y_axis_.reset();
            return false;
        }

        const double vx_map = (x - previous_x) / dt;
        const double vy_map = (y - previous_y) / dt;

        robot_from_map_transform.transform.translation.x = 0.0;
        robot_from_map_transform.transform.translation.y = 0.0;
        robot_from_map_transform.transform.translation.z = 0.0;

        tf2::Transform robot_from_map;
        tf2::fromMsg(robot_from_map_transform.transform, robot_from_map);
        const tf2::Vector3 measured_robot = robot_from_map * tf2::Vector3(vx_map, vy_map, 0.0);
        measured_x = measured_robot.x();
        measured_y = measured_robot.y();
        return true;
    }

    void limitAcceleration(geometry_msgs::Twist& output, double dt) {
        if (accel_limit_ <= 0.0 || dt <= 0.0) {
            rememberOutput(output);
            return;
        }

        if (!has_output_) {
            rememberOutput(output);
            return;
        }

        double delta_x = output.linear.x - last_output_.linear.x;
        double delta_y = output.linear.y - last_output_.linear.y;
        limitVector(delta_x, delta_y, accel_limit_ * dt);
        output.linear.x = last_output_.linear.x + delta_x;
        output.linear.y = last_output_.linear.y + delta_y;
        last_output_ = output;
    }

    void rememberOutput(const geometry_msgs::Twist& output) {
        last_output_ = output;
        has_output_ = true;
    }

    AxisState x_axis_;
    AxisState y_axis_;
    bool enabled_;
    double feedforward_;
    double integral_limit_;
    double output_limit_;
    double accel_limit_;
    double deadband_;
    double feedback_timeout_;
    bool reset_on_stop_;
    double stop_epsilon_;

    bool has_pose_;
    ros::Time last_pose_stamp_;
    double last_pose_x_ = 0.0;
    double last_pose_y_ = 0.0;

    bool has_output_;
    geometry_msgs::Twist last_output_;
};

}  // namespace bot_sim