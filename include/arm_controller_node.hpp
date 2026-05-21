#ifndef ARM_CONTROLLER_NODE_HPP
#define ARM_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

namespace RoboticArm
{

enum class ArmState
{
  IDLE,
  RECEIVING,
  PLANNING,
  MOVING,
  HOLDING,
  COMPLETED,
  ERROR
};

enum class GripperState
{
  OPEN,
  CLOSED,
  HOLDING
};

enum class CommandType
{
  PICK,
  PLACE,
  HOLD,
  RELEASE,
  HOME,
  STOP,
  UNKNOWN
};

struct JointPositions
{
  double base_rotator = 0.0;
  double shoulder_pitch = 0.0;
  double elbow_pitch = 0.0;
  double wrist_pitch = 0.0;
  double wrist_roll = 0.0;
  double gripper = 0.04;

  std::vector<double> to_vector() const
  {
    return {base_rotator, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll};
  }
};

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode();

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void command_callback(const std_msgs::msg::String::SharedPtr msg);
  CommandType parse_command(const std::string& command);
  std::vector<double> parse_position(const std::string& command);
  bool validate_joint_position(const std::string& joint, double position) const;
  void publish_joint_command(const std::string& joint_name, double position);
  void start_async_execution(const JointPositions& target,
    GripperState gripper_action, ArmState completion_state, bool has_gripper_action);
  void execution_callback();

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_cmd_pubs_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr execution_timer_;

  ArmState current_state_;
  GripperState gripper_state_;
  JointPositions current_joint_positions_;
  JointPositions target_joint_positions_;
  std::vector<double> current_joint_velocities_;

  bool is_moving_;
  rclcpp::Time last_command_time_;

  int execution_step_;
  int execution_wait_ticks_;
  ArmState pending_completion_state_;
  GripperState pending_gripper_state_;
  bool pending_has_gripper_action_;
};

}

#endif
