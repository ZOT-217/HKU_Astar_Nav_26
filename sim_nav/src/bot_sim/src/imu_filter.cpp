#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string.h>
#include <cmath>

// ============================================================================
// imu_filter
//
// 历史背景：原版本对每条 /livox/imu 做两步硬编码处理：
//   1) linear_acceleration.{x,y,z} *= -gravity （单位换算 g->m/s² 同时取反，
//      对应旧机器人雷达朝上 / IMU 倒装）。
//   2) 绕 X 轴旋转 -20° （旧机器人雷达机械倾角）。
//
// 现在改为 ROS 参数化，**默认值与旧行为完全一致**，保证旧机器人不受影响。
// 新机器人（雷达 180° 翻转）可通过 launch 覆盖以下参数：
//   accel_unit_scale         : 线加速度统一系数 (默认 9.81，单位换算)
//   accel_invert_x/y/z       : 各轴是否取反 (默认 true,true,true，匹配旧装)
//   tilt_x_deg               : 绕 X 轴附加旋转角 (默认 -20.0)
//   apply_tilt_to_accel      : 是否对线加速度施加 tilt (默认 true)
//   apply_tilt_to_gyro       : 是否对角速度施加 tilt   (默认 true)
//   gravity (旧参数)          : 兼容旧 launch；若设置则覆盖 accel_unit_scale
// ============================================================================

ros::Publisher imu_pub;

static std::string output_frame_id;
static double accel_unit_scale = 9.81;
static bool   accel_invert_x = true;
static bool   accel_invert_y = true;
static bool   accel_invert_z = true;
static double tilt_x_rad = -20.0 * M_PI / 180.0;
static bool   apply_tilt_to_accel = true;
static bool   apply_tilt_to_gyro  = true;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    sensor_msgs::Imu imu_msg = *msg;
    if (!output_frame_id.empty()) {
        imu_msg.header.frame_id = output_frame_id;
    }

    // 1) 线加速度统一系数 + 符号翻转
    double sx = accel_invert_x ? -1.0 : 1.0;
    double sy = accel_invert_y ? -1.0 : 1.0;
    double sz = accel_invert_z ? -1.0 : 1.0;
    imu_msg.linear_acceleration.x *= accel_unit_scale * sx;
    imu_msg.linear_acceleration.y *= accel_unit_scale * sy;
    imu_msg.linear_acceleration.z *= accel_unit_scale * sz;

    // 2) 绕 X 轴旋转 tilt_x_rad（保持与旧实现完全等价的矩阵）
    const double cos_a = std::cos(tilt_x_rad);
    const double sin_a = std::sin(tilt_x_rad);

    if (apply_tilt_to_accel) {
        double y0 = imu_msg.linear_acceleration.y;
        double z0 = imu_msg.linear_acceleration.z;
        imu_msg.linear_acceleration.y =  y0 * cos_a + z0 * sin_a;
        imu_msg.linear_acceleration.z = -y0 * sin_a + z0 * cos_a;
    }
    if (apply_tilt_to_gyro) {
        double y_ang = imu_msg.angular_velocity.y;
        double z_ang = imu_msg.angular_velocity.z;
        imu_msg.angular_velocity.y =  y_ang * cos_a + z_ang * sin_a;
        imu_msg.angular_velocity.z = -y_ang * sin_a + z_ang * cos_a;
    }

    imu_pub.publish(imu_msg);
}

int main(int argc, char **argv) {
    std::string node_name = "imu_filter";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    std::string input_imu_topic;
    std::string output_imu_topic;
    if (!nh.getParam("/" + node_name + "/input_imu_topic", input_imu_topic)) {
        ROS_ERROR("Failed to retrieve parameter 'input_imu_topic'");
        return -1;
    }
    if (!nh.getParam("/" + node_name + "/output_imu_topic", output_imu_topic)) {
        ROS_ERROR("Failed to retrieve parameter 'output_imu_topic'");
        return -1;
    }

    // 兼容旧参数：若设置了 gravity，则用它作为 accel_unit_scale 默认值
    double legacy_gravity = 9.81;
    bool have_legacy_gravity = nh.getParam("/" + node_name + "/gravity", legacy_gravity);

    nh.param<std::string>("/" + node_name + "/output_frame_id", output_frame_id, std::string(""));
    nh.param<double>("/" + node_name + "/accel_unit_scale", accel_unit_scale,
                     have_legacy_gravity ? legacy_gravity : 9.81);
    nh.param<bool>("/" + node_name + "/accel_invert_x", accel_invert_x, true);
    nh.param<bool>("/" + node_name + "/accel_invert_y", accel_invert_y, true);
    nh.param<bool>("/" + node_name + "/accel_invert_z", accel_invert_z, true);
    double tilt_x_deg = -20.0;
    nh.param<double>("/" + node_name + "/tilt_x_deg", tilt_x_deg, -20.0);
    tilt_x_rad = tilt_x_deg * M_PI / 180.0;
    nh.param<bool>("/" + node_name + "/apply_tilt_to_accel", apply_tilt_to_accel, true);
    nh.param<bool>("/" + node_name + "/apply_tilt_to_gyro",  apply_tilt_to_gyro,  true);

    ROS_INFO("imu_filter: scale=%.3f invert=(%d,%d,%d) tilt_x=%.2f deg accel=%d gyro=%d",
             accel_unit_scale, accel_invert_x, accel_invert_y, accel_invert_z,
             tilt_x_deg, apply_tilt_to_accel, apply_tilt_to_gyro);

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(input_imu_topic, 10, imuCallback);
    imu_pub = nh.advertise<sensor_msgs::Imu>(output_imu_topic, 10);

    ros::spin();
    return 0;
}