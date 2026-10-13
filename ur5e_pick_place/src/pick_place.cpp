#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

using moveit::core::MoveItErrorCode;
using moveit::planning_interface::MoveGroupInterface;

namespace
{
const char * kTool = "tool0";
const double kTcpOffset = 0.12;  // tool0 -> gripper_tcp (voir simple_gripper.xacro)
const double kOpenGap = 0.04;    // écart entre les doigts quand le gripper est ouvert
  const double place_margin = node->declare_parameter("place_margin", 0.004);

// Outil orienté vers le bas (rotation de pi autour de x)
geometry_msgs::msg::Pose tool_down_pose(double x, double y, double tool0_z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = tool0_z;
  p.orientation.x = 1.0;
  p.orientation.y = 0.0;
  p.orientation.z = 0.0;
  p.orientation.w = 0.0;
  return p;
}

bool plan_and_execute(MoveGroupInterface & group)
{
  MoveGroupInterface::Plan plan;
  if (group.plan(plan) != MoveItErrorCode::SUCCESS) {
    return false;
  }
  return group.execute(plan) == MoveItErrorCode::SUCCESS;
}

bool go_to_pose(MoveGroupInterface & arm, const geometry_msgs::msg::Pose & pose)
{
  arm.setStartStateToCurrentState();
  arm.setPoseTarget(pose, kTool);
  return plan_and_execute(arm);
}

bool go_linear(
  MoveGroupInterface & arm, const geometry_msgs::msg::Pose & pose, const rclcpp::Logger & logger)
{
  arm.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = arm.computeCartesianPath({pose}, 0.005, 0.0, trajectory);
  if (fraction < 0.99) {
    RCLCPP_ERROR(logger, "Cartesian path only %.0f%% feasible", fraction * 100.0);
    return false;
  }
  return arm.execute(trajectory) == MoveItErrorCode::SUCCESS;
}

bool set_gripper(MoveGroupInterface & gripper, double position)
{
  const auto current = gripper.getCurrentJointValues();
  if (!current.empty() && std::abs(current[0] - position) < 1e-3) {
    return true;  // déjà à la bonne position
  }
  gripper.setStartStateToCurrentState();
  gripper.setJointValueTarget("gripper_finger_left_joint", position);
  return plan_and_execute(gripper);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_place");
  auto logger = node->get_logger();

  const double cube_size = node->declare_parameter("cube_size", 0.03);
  const double cube_x = node->declare_parameter("cube_x", -0.45);
  const double cube_y = node->declare_parameter("cube_y", 0.0);
  const double table_top_z = node->declare_parameter("table_top_z", -0.005);
  const double cube_gap = node->declare_parameter("cube_gap", 0.001);
  const double place_dx = node->declare_parameter("place_dx", 0.0);
  const double place_dy = node->declare_parameter("place_dy", 0.25);
  const double approach_height = node->declare_parameter("approach_height", 0.12);
  const double speed = node->declare_parameter("speed", 0.3);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  MoveGroupInterface arm(node, "ur_manipulator");
  MoveGroupInterface gripper(node, "gripper");
  arm.setPlanningTime(10.0);
  arm.setMaxVelocityScalingFactor(speed);
  arm.setMaxAccelerationScalingFactor(speed);
  gripper.setMaxVelocityScalingFactor(1.0);
  gripper.setMaxAccelerationScalingFactor(1.0);

  // Géométrie du pick and place
  const double cube_center_z = table_top_z + cube_gap + cube_size / 2.0;
  // Bout des doigts à la hauteur du centre du cube (ils dépassent le TCP de 1 cm)
  const double grasp_tool0_z = cube_center_z + 0.01 + kTcpOffset;
  const double above_tool0_z = grasp_tool0_z + approach_height;
  const double closed_position = (kOpenGap - cube_size) / 2.0;

  const auto pre_grasp = tool_down_pose(cube_x, cube_y, above_tool0_z);
  const auto grasp = tool_down_pose(cube_x, cube_y, grasp_tool0_z);
  const auto pre_place = tool_down_pose(cube_x + place_dx, cube_y + place_dy, above_tool0_z);
  const auto place = tool_down_pose(cube_x + place_dx, cube_y + place_dy, grasp_tool0_z + place_margin);

  bool ok = true;
  auto step = [&](const std::string & name, auto && action) {
      if (!ok) {
        return;
      }
      RCLCPP_INFO(logger, "Step: %s", name.c_str());
      if (!action()) {
        RCLCPP_ERROR(logger, "Step FAILED: %s", name.c_str());
        ok = false;
      }
    };
  auto pause = []() {rclcpp::sleep_for(std::chrono::milliseconds(500));};

  step("open gripper", [&]() {return set_gripper(gripper, 0.0);});
  step("move above cube", [&]() {return go_to_pose(arm, pre_grasp);});
  step("descend to cube", [&]() {return go_linear(arm, grasp, logger);});
  step("attach cube", [&]() {
      const bool r = arm.attachObject(
        "cube", "gripper_palm", {"gripper_palm", "gripper_finger_left", "gripper_finger_right"});
      pause();
      return r;
    });
  step("close gripper", [&]() {return set_gripper(gripper, closed_position);});
  step("lift cube", [&]() {return go_linear(arm, pre_grasp, logger);});
  step("move above place position", [&]() {return go_to_pose(arm, pre_place);});
  step("descend to place position", [&]() {return go_linear(arm, place, logger);});
  step("open gripper", [&]() {return set_gripper(gripper, 0.0);});
  step("detach cube", [&]() {
      const bool r = arm.detachObject("cube");
      pause();
      return r;
    });
  step("retreat", [&]() {return go_linear(arm, pre_place, logger);});
  step("return home", [&]() {
      arm.setStartStateToCurrentState();
      arm.setNamedTarget("home");
      return plan_and_execute(arm);
    });

  if (!ok) {
    RCLCPP_WARN(logger, "Run failed: detaching the cube to leave the scene clean.");
    arm.detachObject("cube");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  if (ok) {
    RCLCPP_INFO(logger, "Pick and place finished.");
  }
  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}