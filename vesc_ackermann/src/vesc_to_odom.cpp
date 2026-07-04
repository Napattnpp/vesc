// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <cmath>
#include <string>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  odom_frame_("odom"),
  base_frame_("base_link"),
  heading_source_("servo"),
  publish_tf_(false),
  x_(0.0),
  y_(0.0),
  yaw_(0.0),
  last_yaw_rad_(0.0),
  last_imu_yaw_(0.0),
  initial_imu_yaw_(0.0),
  last_imu_angular_vel_z_(0.0),
  got_initial_imu_(false)
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  heading_source_ = declare_parameter("heading_source", heading_source_);
  
  declare_parameter<double>("speed_to_erpm_gain", 0.0);
  declare_parameter<double>("speed_to_erpm_offset", 0.0);

  speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
  speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();

  if (heading_source_ == "servo") {
    declare_parameter<double>("steering_angle_to_servo_gain", 0.0);
    declare_parameter<double>("steering_angle_to_servo_offset", 0.0);
    declare_parameter<double>("wheelbase", 0.0);
    
    steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
    steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();
    wheelbase_ = get_parameter("wheelbase").get_value<double>();
  }

  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  // subscribe to vesc state and, optionally, servo command or IMU data
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));

  if (heading_source_ == "servo") {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, _1));
  } else if (heading_source_ == "imu") {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10, std::bind(&VescToOdom::imuCallback, this, _1));
  }
}

void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (heading_source_ == "servo" && !last_servo_cmd_) {
    return;
  }

  // If using IMU, wait for first reading
  if (heading_source_ == "imu" && !got_initial_imu_) {
    return;
  }

  // convert to engineering units
  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  if (heading_source_ == "servo") {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_) {
    if (heading_source_ == "imu") {
      initial_imu_yaw_ = last_imu_yaw_;
      got_initial_imu_ = true;
      yaw_ = 0.0;
      last_yaw_rad_ = 0.0;
    }
    last_state_ = state;
    return;
  }

  // calc elapsed time
  auto dt = rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp);

  /** @todo could probably do better propigating odometry, e.g. trapezoidal integration */

  if (heading_source_ == "imu") {
    double current_yaw = last_imu_yaw_ - initial_imu_yaw_;
    // normalize angle
    current_yaw = atan2(sin(current_yaw), cos(current_yaw));
    
    current_angular_velocity = last_imu_angular_vel_z_;
    yaw_ = current_yaw;
    last_yaw_rad_ = current_yaw;
  }

  // propigate odometry
  double x_dot = current_speed * cos(yaw_);
  double y_dot = current_speed * sin(yaw_);
  x_ += x_dot * dt.seconds();
  y_ += y_dot * dt.seconds();
  
  if (heading_source_ == "servo") {
    yaw_ += current_angular_velocity * dt.seconds();
    // normalize yaw
    yaw_ = atan2(sin(yaw_), cos(yaw_));
  } else if (heading_source_ == "none") {
    yaw_ = 0.0;
    current_angular_velocity = 0.0;
  }

  // save state for next time
  last_state_ = state;

  // publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = state->header.stamp;
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

  // Position uncertainty
  /** @todo Think about position uncertainty, perhaps get from parameters? */
  odom.pose.covariance[0] = 0.2;   ///< x
  odom.pose.covariance[7] = 0.2;   ///< y
  odom.pose.covariance[35] = 0.4;  ///< yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = current_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = current_angular_velocity;

  // Velocity uncertainty
  /** @todo Think about velocity uncertainty */

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = now();
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

void VescToOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  auto q = msg->orientation;
  last_imu_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  last_imu_angular_vel_z_ = msg->angular_velocity.z;
  if (!got_initial_imu_) {
    initial_imu_yaw_ = last_imu_yaw_;
    got_initial_imu_ = true;
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
