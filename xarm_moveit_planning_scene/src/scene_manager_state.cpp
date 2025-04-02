#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace yasmin;

/**
 * @brief Represents the "SceneManagerState" state in the state machine.
 *
 * This state is responsible for managing the planning scene in MoveIt.
 * It can add or remove collision objects from the planning scene,
 * and retrieve object IDs from the planning scene.
 */
class SceneManagerState : public yasmin::State {
public:
  bool add_collision_object;
  bool remove_collision_object;
  bool get_objects;
  std::vector<std::string> object_ids;
  moveit_msgs::msg::CollisionObject collision_object;

  /**
   * @brief Constructs a SceneManagerState object.
   */
  SceneManagerState(bool add_collision_object = false, 
                    bool remove_collision_object = false, 
                    bool get_objects = false,
                    std::vector<std::string> object_ids = std::vector<std::string>(),
                    moveit_msgs::msg::CollisionObject collision_object = moveit_msgs::msg::CollisionObject()
  ): yasmin::State({"success"}),
    add_collision_object(add_collision_object),
    remove_collision_object(remove_collision_object),
    get_objects(get_objects),
    object_ids(object_ids),
    collision_object(collision_object)
  {}

  /**
   * @brief Executes the state logic.
   *
   * This method is called when the state machine reaches this state.
   * It manages the planning scene in MoveIt by adding or removing
   * collision objects, or retrieving object IDs from the planning scene.
   *
   * @param blackboard Shared pointer to the blackboard for state communication.
   * @return std::string The outcome of the execution: "succees".
   */
  std::string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    // YASMIN_LOG_INFO("Executing state SceneManagerState");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // Add object to the planning scene
    if (this->add_collision_object) {
      planning_scene_interface_.applyCollisionObject(this->collision_object);
    }
    // Remove object from the planning scene
    else if (this->remove_collision_object) {
      planning_scene_interface_.removeCollisionObjects(this->object_ids);
      // YASMIN_LOG_INFO("Removed collision objects");
    }
    // Get object IDs from the planning scene and remove them or store them in the blackboard
    if (this->get_objects) {
      std::vector<std::string> object_ids = planning_scene_interface_.getKnownObjectNames();
      if (this->remove_collision_object) {
        planning_scene_interface_.removeCollisionObjects(object_ids);
      }
      else {
        blackboard->set<std::vector<std::string>>("object_ids", object_ids);
        // YASMIN_LOG_INFO("Stored object IDs in blackboard, ids: " + object_ids);
      }
    }

    return "success";
  }
};


/**
 * @brief Main function that initializes the ROS 2 node and state machine.
 *
 * This function sets up the state machine, adds states, and handles
 * the execution flow, including logging and cleanup.
 *
 * @param argc Argument count from the command line.
 * @param argv Argument vector from the command line.
 * @return int Exit status of the program. Returns 0 on success.
 *
 * @throws std::exception If there is an error during state machine execution.
 */
int main(int argc, char *argv[]) {
  // YASMIN_LOG_INFO("moveit_yasmin_demo");
  rclcpp::init(argc, argv);

  // Set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // Create collision object for the robot to avoid
  auto const collision_object_ = [frame_id = "world"] 
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;
    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;
    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.3;
    box_pose.position.z = 0.35;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(std::initializer_list<std::string>{"end"});
  // Add states to the state machine
  sm->add_state("BAR", std::make_shared<SceneManagerState>(true, false, false, std::vector<std::string>(), collision_object_),
                {
                    {"success", "remove"},
                });
  sm->add_state("remove", std::make_shared<SceneManagerState>(false, true, true),
                {
                    {"success", "end"},
                });

  // Execute the state machine
  try {
    std::string outcome = (*sm.get())();
    // YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    // YASMIN_LOG_WARN(e.what());
  }

  // Cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  rclcpp::shutdown();

  return 0;
}