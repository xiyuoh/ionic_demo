// Copyright 2024 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <gz/sim/System.hh>

#include <chrono>

const std::string MOVE_GROUP = "arm";
const std::string GRIPPER_GROUP = "gripper";

using namespace gz::sim;
using namespace std::chrono_literals;

/* PRE_GRASP
 * -11
 * 17
 * -2
 *  -114
 *  1
 *  130
 *  0
*/


/* GRASP
 * -11
 * 27
 * 0
 *  -116
 *  0
 *  142
 *  0
*/

/* PRE_DROP
 * 42
 * 36
 * 40
 * -115
 *  -34
 *  138
 *  0
*/

/* DROP
 * 59
 * 72
 * 20
 * -92
 *  -50
 *  155
 *  0
*/

class MoveItFollowTarget : public rclcpp::Node
{
public:
  /// Constructor
  MoveItFollowTarget();

private:
  void dispenser_request_callback(const rmf_dispenser_msgs::msg::DispenserRequest::ConstSharedPtr msg);

  void timer_callback();
  void publish_dispenser_state(bool busy);

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;
  /// Move group interface for the gripper
  moveit::planning_interface::MoveGroupInterface gripper_move_group_;

  /// RMF dispenser interfaces
  rclcpp::Subscription<rmf_dispenser_msgs::msg::DispenserRequest>::SharedPtr dispenser_request_sub_;
  rclcpp::Publisher<rmf_dispenser_msgs::msg::DispenserState>::SharedPtr dispenser_state_pub_;

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher attaching_pub_;
  gz::transport::Node::Publisher detaching_pub_;
  bool attach_requested_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
};

MoveItFollowTarget::MoveItFollowTarget() : Node("ex_follow_target"),
                                           move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP),
                                           gripper_move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), GRIPPER_GROUP)
{
  // Use upper joint velocity and acceleration limits
  this->move_group_.setMaxAccelerationScalingFactor(1.0);
  this->move_group_.setMaxVelocityScalingFactor(1.0);
  // TODO(luca) Add the poses to the robot here instead of the srdf
  attaching_pub_ = gz_node_.Advertise<gz::msgs::Empty>("/panda/attach");
  detaching_pub_ = gz_node_.Advertise<gz::msgs::Empty>("/panda/detach");

  dispenser_state_pub_ = this->create_publisher<rmf_dispenser_msgs::msg::DispenserState>("/dispenser_states", rclcpp::QoS(10));
  dispenser_request_sub_ = this->create_subscription<rmf_dispenser_msgs::msg::DispenserRequest>("/dispenser_requests", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::dispenser_request_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(500ms, std::bind(&MoveItFollowTarget::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MoveItFollowTarget::timer_callback()
{
  gz::msgs::Empty req;
  if (!this->attach_requested_)
  {
    detaching_pub_.Publish(req);
  }
  // Also publish a dispenser state
  this->publish_dispenser_state(false);
}

void MoveItFollowTarget::publish_dispenser_state(bool busy)
{
  rmf_dispenser_msgs::msg::DispenserState msg;
  msg.guid = "moveit_dispenser";
  msg.mode = busy ? msg.BUSY : msg.IDLE;
  dispenser_state_pub_->publish(msg);
}

void MoveItFollowTarget::dispenser_request_callback(const rmf_dispenser_msgs::msg::DispenserRequest::ConstSharedPtr msg)
{
  if (msg->target_guid != "moveit_dispenser")
    return;
  this->publish_dispenser_state(true);
  gz::msgs::Empty req;
  RCLCPP_INFO(this->get_logger(), "Received dispenser request");
  const auto ready = this->move_group_.getNamedTargetValues("ready");
  this->move_group_.setJointValueTarget(ready);
  this->move_group_.move();
  const auto open = this->gripper_move_group_.getNamedTargetValues("open");
  this->gripper_move_group_.setJointValueTarget(open);
  this->gripper_move_group_.move();
  const auto pregrasp = this->move_group_.getNamedTargetValues("pre_grasp");
  this->move_group_.setJointValueTarget(pregrasp);
  this->move_group_.move();
  const auto grasp = this->move_group_.getNamedTargetValues("grasp");
  this->move_group_.setJointValueTarget(grasp);
  this->move_group_.move();
  const auto close = this->gripper_move_group_.getNamedTargetValues("close");
  this->gripper_move_group_.setJointValueTarget(close);
  this->gripper_move_group_.move();
  attaching_pub_.Publish(req);
  this->attach_requested_ = true;
  this->move_group_.setJointValueTarget(ready);
  this->move_group_.move();
  const auto pre_drop = this->move_group_.getNamedTargetValues("pre_drop");
  this->move_group_.setJointValueTarget(pre_drop);
  this->move_group_.move();
  const auto drop = this->move_group_.getNamedTargetValues("drop");
  this->move_group_.setJointValueTarget(drop);
  this->move_group_.move();
  this->gripper_move_group_.setJointValueTarget(open);
  this->gripper_move_group_.move();
  this->attach_requested_ = false;
  detaching_pub_.Publish(req);
  this->move_group_.setJointValueTarget(pre_drop);
  this->move_group_.move();
  this->move_group_.setJointValueTarget(ready);
  this->move_group_.move();
  this->publish_dispenser_state(false);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto target_follower = std::make_shared<MoveItFollowTarget>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(target_follower);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
