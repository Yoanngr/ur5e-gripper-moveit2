#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

moveit_msgs::msg::CollisionObject make_box(
  const std::string & id, const std::string & frame,
  double sx, double sy, double sz, double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = frame;
  obj.id = id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {sx, sy, sz};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("scene_setup");

  // Paramètres modifiables en ligne de commande (--ros-args -p cube_x:=0.4)
  const double cube_size = node->declare_parameter("cube_size", 0.03);
  const double cube_x = node->declare_parameter("cube_x", -0.45);
  const double cube_y = node->declare_parameter("cube_y", 0.0);
  const double table_top_z = node->declare_parameter("table_top_z", -0.005);
  const double table_thickness = 0.04;
  const double cube_gap = node->declare_parameter("cube_gap", 0.001);

  moveit::planning_interface::PlanningSceneInterface scene;

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.push_back(make_box(
    "table", "base_link", 1.2, 1.2, table_thickness,
    0.0, 0.0, table_top_z - table_thickness / 2.0));
  objects.push_back(make_box(
    "cube", "base_link", cube_size, cube_size, cube_size,
    cube_x, cube_y, table_top_z + cube_size / 2.0));

  scene.applyCollisionObjects(objects);
  RCLCPP_INFO(node->get_logger(), "Table and cube added to the planning scene.");

  rclcpp::shutdown();
  return 0;
}