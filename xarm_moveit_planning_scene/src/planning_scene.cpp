#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "xarm6");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Set a target Pose with updated values !!!
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.y = 0.8;
    msg.orientation.w = 0.6;
    msg.position.x = 0.15;
    msg.position.y = 0.4;
    msg.position.z = 0.35;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create collisions objects for the robot to avoid
    std::string frame_id = move_group_interface.getPlanningFrame();
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    std::vector<moveit_msgs::msg::ObjectColor> colors;
    for (int i = 1; i < 5; ++i) // Adjust the range as needed
    {
      moveit_msgs::msg::CollisionObject object;
      moveit_msgs::msg::ObjectColor color;
      // set the color for each object
      color.color.r = 0.2 * i;
      color.color.g = 0.4 * i;
      color.color.b = 0.8 * i;
      color.color.a = 1.0;
      colors.push_back(color);
      object.header.frame_id = frame_id;
      object.id = "box" + std::to_string(i);
      shape_msgs::msg::SolidPrimitive primitive;

      // Define the size of the box in meters
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 0.5;
      primitive.dimensions[primitive.BOX_Y] = 0.1;
      primitive.dimensions[primitive.BOX_Z] = 0.2;

      // Define the pose of the box (relative to the frame_id)
      geometry_msgs::msg::Pose box_pose;
      box_pose.orientation.w = 1.0;
      box_pose.position.x = 0.2;
      box_pose.position.y = 0.2 * i;
      box_pose.position.z = 0.25;

      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(box_pose);
      object.operation = object.ADD;

      objects.push_back(object);
    }

  // Add the collision objects to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(objects, colors);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    // draw_trajectory_tool_path(plan.trajectory);
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  }
  else
  {
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}