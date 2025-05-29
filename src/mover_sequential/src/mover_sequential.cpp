#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>  // <---- add this to the set of includes at the top
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <regex>

// constexpr double M_PI = 3.14159265358979323846;

double x = 0.0, y = 0.0, z=0;
double theta = M_PI / 2; // 90 degrees

// parse a gcode file and return a vector with poses to be used for cartesian path planning
std::vector<geometry_msgs::msg::Pose> parseGcodeFile(const std::string &filename) {
    std::vector<geometry_msgs::msg::Pose> poses;
    std::ifstream gcodeFile(filename);

    if (!gcodeFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return poses;
    }

    std::string line;
    std::regex coordinateRegex(R"(X([-\d\.]+)\s*Y([-\d\.]+)\s*Z([-\d\.]+)?)");

    while (std::getline(gcodeFile, line)) {
        // Ignore comments or empty lines
        if (line.empty() || line[0] == ';') {
            continue;
        }

        std::istringstream stream(line);
        std::string token;
        
        bool has_x = false, has_y = false, has_z = false;

        while (stream >> token) {
            // Parse tokens for X, Y, Z
            if (token[0] == 'X') {
                x = std::stod(token.substr(1));
                has_x = true;
            } else if (token[0] == 'Y') {
                y = std::stod(token.substr(1));
                has_y = true;
            } else if (token[0] == 'Z') {
                z = std::stod(token.substr(1));
                has_z = true;
            }
        }

        // Create a Pose if we have at least X and Y
        if (has_x || has_y || has_z) {

          

            geometry_msgs::msg::Pose pose;
            pose.position.x = x/1000;
            pose.position.y = y/1000;
            pose.position.z = z/1000 + 0.25;
            pose.orientation.x = 0.0;
            pose.orientation.y = sin(theta / 2);
            pose.orientation.z = 0.0; // Axis is Z
            pose.orientation.w = cos(theta / 2);

            poses.push_back(pose);
        }
    }

    gcodeFile.close();
    return poses;
}


int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "mover_sequential",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("mover_sequential");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // parse arguments to get the file
  if (argc != 2) {
    RCLCPP_ERROR(logger, "Usage: mover_sequential <gcode_file>");
    return 1;
  }

  // get the poses
  std::string gcodeFilename = argv[1];
  auto poses = parseGcodeFile(gcodeFilename);
  std::cout << "Extracted " << poses.size() << " poses from " << gcodeFilename << std::endl;


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

// Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();


  // Create a closures(function objects that have access to variables in our current scope) for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };

  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          "arm")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  for( auto pose : poses){
    move_group_interface.setPoseTarget(pose);
    auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();
   // Execute the plan
    if (success) {
      draw_trajectory_tool_path(plan.trajectory_);
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  }


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
