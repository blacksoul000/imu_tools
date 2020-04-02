/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

	@section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "imu_complementary_filter/complementary_filter_ros.h"

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

namespace imu_tools {

using std::placeholders::_1;
using std::placeholders::_2;

ComplementaryFilterROS::ComplementaryFilterROS(rclcpp::Node::SharedPtr node):
  node_(node),
  tf_broadcaster_(node),
  initialized_filter_(false)
{
  RCLCPP_INFO(node_->get_logger(), "Starting ComplementaryFilterROS");
  initializeParams();

  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;

  // Register publishers:
  imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());

  if (publish_debug_topics_)
  {
      rpy_publisher_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                  "imu/rpy/filtered", rclcpp::SensorDataQoS());

      if (filter_.getDoBiasEstimation())
      {
        state_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
                    "imu/steady_state", rclcpp::SystemDefaultsQoS());
      }
  }

  // Register IMU raw data subscriber.
  imu_subscriber_ = std::make_shared<ImuSubscriber>(node_, "imu/data_raw", qos);

  // Register magnetic data subscriber.
  if (use_mag_)
  {
    mag_subscriber_ = std::make_shared<MagSubscriber>(node_, "imu/mag", qos);

    const int queue_size = 5;
    sync_ = std::make_shared<Synchronizer>(
        SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_);
    sync_->registerCallback(
        std::bind(&ComplementaryFilterROS::imuMagCallback, this, _1, _2));
  }
  else
  {
    imu_subscriber_->registerCallback(
        &ComplementaryFilterROS::imuCallback, this);
  }
}

ComplementaryFilterROS::~ComplementaryFilterROS()
{
  RCLCPP_INFO(node_->get_logger(), "Destroying ComplementaryFilterROS");
}

void ComplementaryFilterROS::initializeParams()
{
  double gain_acc;
  double gain_mag;
  bool do_bias_estimation;
  double bias_alpha;
  bool do_adaptive_gain;

  fixed_frame_ = node_->declare_parameter("fixed_frame", "odom");
  use_mag_ = node_->declare_parameter("use_mag", true);
  publish_tf_ = node_->declare_parameter("publish_tf", false);
  reverse_tf_ = node_->declare_parameter("reverse_tf", false);
  constant_dt_ = node_->declare_parameter("constant_dt", 0.0);
  publish_debug_topics_ = node_->declare_parameter("publish_debug_topics", false);
  gain_acc = node_->declare_parameter("gain_acc", 0.01);
  gain_mag = node_->declare_parameter("gain_mag", 0.01);
  do_bias_estimation = node_->declare_parameter("do_bias_estimation", true);
  bias_alpha = node_->declare_parameter("bias_alpha", 0.01);
  do_adaptive_gain = node_->declare_parameter("do_adaptive_gain", true);
  double orientation_stddev = node_->declare_parameter("orientation_stddev", 0.0);

  orientation_variance_ = orientation_stddev * orientation_stddev;

  filter_.setDoBiasEstimation(do_bias_estimation);
  filter_.setDoAdaptiveGain(do_adaptive_gain);

  if(!filter_.setGainAcc(gain_acc))
    RCLCPP_WARN(node_->get_logger(), "Invalid gain_acc passed to ComplementaryFilter.");
  if (use_mag_)
  {
    if(!filter_.setGainMag(gain_mag))
      RCLCPP_WARN(node_->get_logger(), "Invalid gain_mag passed to ComplementaryFilter.");
  }
  if (do_bias_estimation)
  {
    if(!filter_.setBiasAlpha(bias_alpha))
      RCLCPP_WARN(node_->get_logger(), "Invalid bias_alpha passed to ComplementaryFilter.");
  }

  // check for illegal constant_dt values
  if (constant_dt_ < 0.0)
  {
    // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
    // otherwise, it will be constant
    RCLCPP_WARN(node_->get_logger(), "constant_dt parameter is %f, must be >= 0.0. Setting to 0.0", constant_dt_);
    constant_dt_ = 0.0;
  }
}

void ComplementaryFilterROS::imuCallback(ImuMsg::SharedPtr imu_msg_raw)
{
  const geometry_msgs::msg::Vector3& a = imu_msg_raw->linear_acceleration;
  const geometry_msgs::msg::Vector3& w = imu_msg_raw->angular_velocity;
  const rclcpp::Time& time = imu_msg_raw->header.stamp;

  // Initialize.
  if (!initialized_filter_)
  {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // determine dt: either constant, or from IMU timestamp
  double dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - time_prev_).seconds();

  time_prev_ = time;

  // Update the filter.
  filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

  // Publish state.
  publish(imu_msg_raw);
}

void ComplementaryFilterROS::imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                                            const MagMsg::ConstPtr& mag_msg)
{
  const geometry_msgs::msg::Vector3& a = imu_msg_raw->linear_acceleration;
  const geometry_msgs::msg::Vector3& w = imu_msg_raw->angular_velocity;
  const geometry_msgs::msg::Vector3& m = mag_msg->magnetic_field;
  const rclcpp::Time& time = imu_msg_raw->header.stamp;

  // Initialize.
  if (!initialized_filter_)
  {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // Calculate dt.
  double dt = (time - time_prev_).seconds();
  time_prev_ = time;
   //ros::Time t_in, t_out;
  //t_in = ros::Time::now();
  // Update the filter.
  if (std::isnan(m.x) || std::isnan(m.y) || std::isnan(m.z))
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
  else
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, m.x, m.y, m.z, dt);

  //t_out = ros::Time::now();
  //float dt_tot = (t_out - t_in).toSec() * 1000.0; // In msec.
  //printf("%.6f\n", dt_tot);
  // Publish state.
  publish(imu_msg_raw);
}

tf2::Quaternion ComplementaryFilterROS::hamiltonToTFQuaternion(
    double q0, double q1, double q2, double q3) const
{
  // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
  // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
  return tf2::Quaternion(q1, q2, q3, q0);
}

void ComplementaryFilterROS::publish(
    const sensor_msgs::msg::Imu::ConstPtr& imu_msg_raw)
{
  // Get the orientation:
  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  tf2::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

  // Create and publish fitlered IMU message.
  std::unique_ptr<ImuMsg> imu_msg = std::make_unique<ImuMsg>(*imu_msg_raw);
  imu_msg->orientation = tf2::toMsg(q);

  imu_msg->orientation_covariance[0] = orientation_variance_;
  imu_msg->orientation_covariance[1] = 0.0;
  imu_msg->orientation_covariance[2] = 0.0;
  imu_msg->orientation_covariance[3] = 0.0;
  imu_msg->orientation_covariance[4] = orientation_variance_;
  imu_msg->orientation_covariance[5] = 0.0;
  imu_msg->orientation_covariance[6] = 0.0;
  imu_msg->orientation_covariance[7] = 0.0;
  imu_msg->orientation_covariance[8] = orientation_variance_;

  // Account for biases.
  if (filter_.getDoBiasEstimation())
  {
    imu_msg->angular_velocity.x -= filter_.getAngularVelocityBiasX();
    imu_msg->angular_velocity.y -= filter_.getAngularVelocityBiasY();
    imu_msg->angular_velocity.z -= filter_.getAngularVelocityBiasZ();
  }

  imu_publisher_->publish(std::move(imu_msg));

  if (publish_debug_topics_)
  {
      // Create and publish roll, pitch, yaw angles
      geometry_msgs::msg::Vector3Stamped rpy;
      rpy.header = imu_msg_raw->header;

      tf2::Matrix3x3 M;
      M.setRotation(q);
      M.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
      rpy_publisher_->publish(rpy);

      // Publish whether we are in the steady state, when doing bias estimation
      if (filter_.getDoBiasEstimation())
      {
        std_msgs::msg::Bool state_msg;
        state_msg.data = filter_.getSteadyState();
        state_publisher_->publish(state_msg);
      }
  }

  if (publish_tf_)
  {
      // Create and publish the ROS tf.
      tf2::Transform transform;
      transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      transform.setRotation(q);

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = imu_msg_raw->header.stamp;

      if (reverse_tf_)
      {
          tf.header.frame_id = fixed_frame_;
          tf.child_frame_id = imu_msg_raw->header.frame_id;
          tf.transform = tf2::toMsg(transform.inverse());
      }
      else
      {
          tf.header.frame_id = imu_msg_raw->header.frame_id;
          tf.child_frame_id = fixed_frame_;
          tf.transform = tf2::toMsg(transform);
      }

      tf_broadcaster_.sendTransform(tf);
  }
}

}  // namespace imu_tools
