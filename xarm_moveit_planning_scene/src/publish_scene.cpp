#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// MoveIt
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("xarm6_planning_scene_with_ros_moveit_api");

int main(int argc, char** argv)
{
  // Create a ROS Node and initialize it
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "xarm6_planning_scene_with_ros_moveit_api", 
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "xarm6");

  // Create the publisher to publish the planning scene
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

  
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "box1";
  attached_object.object.header.frame_id = "world";
  attached_object.object.id = "box";

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.55;
  pose.position.y = 0.0;
  pose.position.z = 0.11;
  pose.orientation.w = 1.0;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.2;
  primitive.dimensions[1] = 0.3;
  primitive.dimensions[2] = 0.2;


  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);
  attached_object.object.operation = attached_object.object.ADD;


  // Add the object to the world by publishing to a topic te planning scene   
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);



  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}

