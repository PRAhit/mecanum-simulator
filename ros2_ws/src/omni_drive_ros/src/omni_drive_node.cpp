// SPDX-License-Identifier: MIT
//
// ROS 2 wrapper around odc::DriveCore.
//
// The node is deliberately thin. It owns ROS concerns only -- topics, TF,
// parameters, the timer -- and contains no control logic whatsoever. Every
// decision is made inside DriveCore::step(), which is the same object the
// Python simulator drives and the same object that would be cross-compiled
// onto the main wheel's MCU.
//
// That split is the point. Control code that can only run inside a ROS graph
// cannot be unit tested at 200 Hz, cannot be replayed deterministically, and
// cannot be moved onto a microcontroller when the product needs it to be.

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "odc/drive_core.hpp"

namespace {

constexpr const char* kWheelNames[odc::kNumWheels] = {
    "wheel_front_left_joint", "wheel_front_right_joint",
    "wheel_rear_left_joint", "wheel_rear_right_joint"};

std::string faultsToString(std::uint32_t f) {
  if (f == odc::kFaultNone) return "none";
  std::string s;
  auto add = [&s](const char* n) {
    if (!s.empty()) s += "|";
    s += n;
  };
  if (f & odc::kFaultCommandTimeout) add("command_timeout");
  if (f & odc::kFaultBusTimeout) add("bus_timeout");
  if (f & odc::kFaultWheelUnhealthy) add("wheel_unhealthy");
  if (f & odc::kFaultWheelSlip) add("wheel_slip");
  if (f & odc::kFaultEstop) add("estop");
  if (f & odc::kFaultObstacle) add("obstacle");
  if (f & odc::kFaultLocalisationLost) add("localisation_lost");
  return s;
}

const char* stateToString(odc::DriveState s) {
  switch (s) {
    case odc::DriveState::kIdle:
      return "IDLE";
    case odc::DriveState::kRunning:
      return "RUNNING";
    case odc::DriveState::kSpeedLimited:
      return "SPEED_LIMITED";
    case odc::DriveState::kProtectiveStop:
      return "PROTECTIVE_STOP";
    case odc::DriveState::kFault:
      return "FAULT";
  }
  return "UNKNOWN";
}

}  // namespace

class OmniDriveNode : public rclcpp::Node {
 public:
  OmniDriveNode() : rclcpp::Node("omni_drive_node") {
    declareAndLoadParameters();
    core_ = std::make_unique<odc::DriveCore>(cfg_);

    using std::placeholders::_1;
    using std::placeholders::_2;
    const auto sensor_qos = rclcpp::SensorDataQoS();

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&OmniDriveNode::onCmdVel, this, _1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "wheel_states", sensor_qos,
        std::bind(&OmniDriveNode::onJointState, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", sensor_qos, std::bind(&OmniDriveNode::onImu, this, _1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", sensor_qos, std::bind(&OmniDriveNode::onScan, this, _1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "estop", 10, std::bind(&OmniDriveNode::onEstop, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    wheel_cmd_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("wheel_commands", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "reset_faults", std::bind(&OmniDriveNode::onReset, this, _1, _2));

    const auto period = std::chrono::duration<double>(cfg_.control_period);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&OmniDriveNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "omni_drive_node up at %.0f Hz",
                1.0 / cfg_.control_period);
  }

 private:
  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  void declareAndLoadParameters() {
    auto d = [this](const std::string& name, double def) {
      return this->declare_parameter<double>(name, def);
    };

    cfg_.control_period = d("control_period", 0.005);
    cfg_.geometry.wheel_radius = d("geometry.wheel_radius", 0.0625);
    cfg_.geometry.half_wheelbase = d("geometry.half_wheelbase", 0.30);
    cfg_.geometry.half_track = d("geometry.half_track", 0.25);

    cfg_.wheel.kp = d("wheel.kp", 0.45);
    cfg_.wheel.ki = d("wheel.ki", 6.0);
    cfg_.wheel.torque_limit = d("wheel.torque_limit", 4.5);
    cfg_.wheel.accel_limit = d("wheel.accel_limit", 40.0);
    cfg_.wheel.anti_windup_gain = d("wheel.anti_windup_gain", 13.3);

    cfg_.tracker.kp_xy = d("tracker.kp_xy", 2.2);
    cfg_.tracker.kp_theta = d("tracker.kp_theta", 3.0);
    cfg_.tracker.max_vx = d("tracker.max_vx", 1.2);
    cfg_.tracker.max_vy = d("tracker.max_vy", 0.9);
    cfg_.tracker.max_wz = d("tracker.max_wz", 1.5);
    cfg_.tracker.max_accel = d("tracker.max_accel", 1.0);

    cfg_.safety.stop_distance = d("safety.stop_distance", 0.35);
    cfg_.safety.slow_distance = d("safety.slow_distance", 1.40);
    cfg_.safety.command_timeout = d("safety.command_timeout", 0.30);
    cfg_.safety.bus_timeout = d("safety.bus_timeout", 0.10);

    cfg_.slip.kin_enter = d("slip.kin_enter", 1.2);
    cfg_.slip.gyro_enter = d("slip.gyro_enter", 0.20);

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    feedback_timeout_ = d("feedback_timeout", 0.05);
  }

  // ---------------------------------------------------------------------------
  // Callbacks. These only latch data; nothing is computed here.
  // ---------------------------------------------------------------------------
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    // A velocity command is integrated into a moving pose reference, so the
    // tracker still closes a position loop rather than running open-loop on
    // velocity -- which is what stops the platform drifting off an aisle
    // centre line over a 40 m run.
    cmd_twist_ = odc::Twist2D{msg->linear.x, msg->linear.y, msg->angular.z};
    cmd_stamp_ = now();
    have_cmd_ = true;
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    for (std::size_t i = 0; i < odc::kNumWheels; ++i) {
      for (std::size_t j = 0; j < msg->name.size(); ++j) {
        if (msg->name[j] != kWheelNames[i]) continue;
        if (j < msg->velocity.size()) feedback_.velocity[i] = msg->velocity[j];
        if (j < msg->effort.size()) feedback_.current[i] = msg->effort[j];
        feedback_.healthy[i] = true;
      }
    }
    feedback_stamp_ = now();
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    gyro_z_ = msg->angular_velocity.z;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double nearest = std::numeric_limits<double>::infinity();
    for (const float r : msg->ranges) {
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        nearest = std::min(nearest, static_cast<double>(r));
      }
    }
    std::lock_guard<std::mutex> lk(mu_);
    nearest_obstacle_ = nearest;
  }

  void onEstop(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    estop_ = msg->data;
  }

  void onReset(const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr res) {
    std::lock_guard<std::mutex> lk(mu_);
    core_->reset();
    res->success = true;
    res->message = "faults cleared";
    RCLCPP_WARN(get_logger(), "faults cleared by service call");
  }

  // ---------------------------------------------------------------------------
  // The control cycle
  // ---------------------------------------------------------------------------
  void onTimer() {
    odc::CycleInput in{};
    rclcpp::Time stamp = now();
    double dt = cfg_.control_period;

    {
      std::lock_guard<std::mutex> lk(mu_);
      if (last_tick_.nanoseconds() != 0) {
        dt = (stamp - last_tick_).seconds();
        // Clamp: a scheduling hiccup must not be integrated as real elapsed
        // time, or the EKF and every integrator take a step change.
        dt = std::clamp(dt, 0.2 * cfg_.control_period,
                        5.0 * cfg_.control_period);
      }
      last_tick_ = stamp;

      const double fb_age = (stamp - feedback_stamp_).seconds();
      const bool fb_ok =
          feedback_stamp_.nanoseconds() != 0 && fb_age < feedback_timeout_;
      in.wheels = feedback_;
      if (!fb_ok) {
        for (std::size_t i = 0; i < odc::kNumWheels; ++i) {
          in.wheels.healthy[i] = false;
        }
      }
      in.gyro_z = gyro_z_;
      in.safety.nearest_obstacle_m = nearest_obstacle_;
      in.safety.estop_engaged = estop_;
      in.safety.bus_ok = fb_ok;

      if (have_cmd_ &&
          (stamp - cmd_stamp_).seconds() < cfg_.safety.command_timeout) {
        integrateReference(cmd_twist_, dt);
        in.reference_fresh = true;
      }
      in.reference = reference_;
    }

    const odc::ControlOutput out = core_->step(in, dt);
    publish(out, stamp);
  }

  /// Advance the pose reference by the commanded body twist.
  void integrateReference(const odc::Twist2D& t, double dt) {
    const double c = std::cos(reference_.pose.theta);
    const double s = std::sin(reference_.pose.theta);
    reference_.pose.x += (t.vx * c - t.vy * s) * dt;
    reference_.pose.y += (t.vx * s + t.vy * c) * dt;
    reference_.pose.theta = odc::wrapAngle(reference_.pose.theta + t.wz * dt);
    reference_.velocity.vx = t.vx * c - t.vy * s;
    reference_.velocity.vy = t.vx * s + t.vy * c;
    reference_.velocity.wz = t.wz;
  }

  void publish(const odc::ControlOutput& out, const rclcpp::Time& stamp) {
    // --- Wheel commands.
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = stamp;
    cmd.name.assign(kWheelNames, kWheelNames + odc::kNumWheels);
    cmd.velocity.assign(out.velocity_ref.begin(), out.velocity_ref.end());
    cmd.effort.assign(out.torque_cmd.begin(), out.torque_cmd.end());
    wheel_cmd_pub_->publish(cmd);

    // --- Odometry.
    tf2::Quaternion q;
    q.setRPY(0, 0, out.estimated_pose.theta);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = out.estimated_pose.x;
    odom.pose.pose.position.y = out.estimated_pose.y;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x = out.estimated_twist.vx;
    odom.twist.twist.linear.y = out.estimated_twist.vy;
    odom.twist.twist.angular.z = out.estimated_twist.wz;
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = out.estimated_pose.x;
      tf.transform.translation.y = out.estimated_pose.y;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    // --- Diagnostics, throttled to 5 Hz.
    if ((stamp - last_diag_).seconds() < 0.2) return;
    last_diag_ = stamp;

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "omni_drive_core";
    st.hardware_id = "mecanum_wheelset";
    st.message = std::string(stateToString(out.state)) +
                 " faults=" + faultsToString(out.faults);
    st.level = (out.state == odc::DriveState::kFault)
                   ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
               : (out.faults != odc::kFaultNone)
                   ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                   : diagnostic_msgs::msg::DiagnosticStatus::OK;

    auto kv = [&st](const std::string& k, const std::string& v) {
      diagnostic_msgs::msg::KeyValue e;
      e.key = k;
      e.value = v;
      st.values.push_back(e);
    };
    kv("speed_scale", std::to_string(out.speed_scale));
    kv("slip_channels", std::to_string(out.slip_channels));
    kv("slip_wheel", out.slip_wheel < odc::kNumWheels
                         ? kWheelNames[out.slip_wheel]
                         : "unknown");

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = stamp;
    arr.status.push_back(st);
    diag_pub_->publish(arr);

    if (out.state == odc::DriveState::kFault) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "FAULT: %s",
                            faultsToString(out.faults).c_str());
    }
  }

  // ---------------------------------------------------------------------------
  odc::DriveCoreConfig cfg_{};
  std::unique_ptr<odc::DriveCore> core_;

  std::mutex mu_;
  odc::WheelFeedback feedback_{};
  odc::TrajectoryPoint reference_{};
  odc::Twist2D cmd_twist_{};
  double gyro_z_ = 0.0;
  double nearest_obstacle_ = std::numeric_limits<double>::infinity();
  bool estop_ = false;
  bool have_cmd_ = false;
  double feedback_timeout_ = 0.05;

  rclcpp::Time cmd_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time feedback_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diag_{0, 0, RCL_ROS_TIME};

  std::string odom_frame_, base_frame_;
  bool publish_tf_ = true;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OmniDriveNode>());
  rclcpp::shutdown();
  return 0;
}
