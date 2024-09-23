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
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <gz/sim/System.hh>

const std::string MOVE_GROUP = "arm";
const std::string GRIPPER_GROUP = "gripper";

using namespace gz::sim;

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
class MoveItFollowTarget : public rclcpp::Node
{
public:
  /// Constructor
  MoveItFollowTarget();

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;
  /// Move group interface for the gripper
  moveit::planning_interface::MoveGroupInterface gripper_move_group_;
  /// Subscriber for dispenser requests
  rclcpp::Subscription<rmf_dispenser_msgs::msg::DispenserRequest>::SharedPtr dispenser_request_sub_;
private:
  void dispenser_request_callback(const rmf_dispenser_msgs::msg::DispenserRequest::ConstSharedPtr msg);

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher attaching_pub_;
  gz::transport::Node::Publisher detaching_pub_;
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

  dispenser_request_sub_ = this->create_subscription<rmf_dispenser_msgs::msg::DispenserRequest>("/dispenser_request", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::dispenser_request_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MoveItFollowTarget::dispenser_request_callback(const rmf_dispenser_msgs::msg::DispenserRequest::ConstSharedPtr msg)
{
  // TODO(luca) uncomment this when we have full dispenser / ingestors
  /*
  if (msg->target_guid != "moveit_dispenser")
    return;
  */
  gz::msgs::Empty req;
  RCLCPP_INFO(this->get_logger(), "Received dispenser request");
  detaching_pub_.Publish(req);
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
  this->move_group_.setJointValueTarget(ready);
  this->move_group_.move();
  // TODO(luca) Create a drop pose in srdf and uncomment this
  /*
  const auto drop = this->move_group_.getNamedTargetValues("drop");
  this->move_group_.setJointValueTarget(drop);
  this->move_group_.move();
  */
  this->gripper_move_group_.setJointValueTarget(open);
  this->gripper_move_group_.move();
  detaching_pub_.Publish(req);
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
