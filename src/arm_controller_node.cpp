#include "arm_controller_node.hpp"
#include <chrono>

namespace RoboticArm
{

static const char * MODEL = "robotic_arm";

ArmControllerNode::ArmControllerNode()
: Node("arm_controller"),
  current_state_(ArmState::IDLE),
  gripper_state_(GripperState::OPEN),
  is_moving_(false),
  execution_step_(-1),
  execution_wait_ticks_(0),
  pending_completion_state_(ArmState::IDLE),
  pending_gripper_state_(GripperState::OPEN),
  pending_has_gripper_action_(false)
{
  RCLCPP_INFO(this->get_logger(), "Robotic Arm Controller Node initialized");

  this->declare_parameter<bool>("use_sim_time", true);

  current_joint_positions_ = JointPositions();
  target_joint_positions_ = JointPositions();

  const std::vector<std::string> joints = {
    "base_rotator", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll",
    "gripper_joint", "gripper_joint_right",
  };
  for (const auto & joint : joints) {
    std::string topic = std::string("/model/") + MODEL + "/joint/" + joint + "/cmd_pos";
    joint_cmd_pubs_[joint] = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
  }

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&ArmControllerNode::joint_state_callback, this, std::placeholders::_1));

  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/arm_controller/commands", 10,
    std::bind(&ArmControllerNode::command_callback, this, std::placeholders::_1));

  last_command_time_ = this->now();

  status_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_controller/status", 10);

  home_service_ = this->create_service<std_srvs::srv::Empty>(
    "/arm_controller/home",
    [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
           std::shared_ptr<std_srvs::srv::Empty::Response>) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (execution_step_ < 0) {
        start_async_execution(JointPositions(), GripperState::OPEN, ArmState::IDLE, false);
      }
    });

  stop_service_ = this->create_service<std_srvs::srv::Empty>(
    "/arm_controller/stop",
    [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
           std::shared_ptr<std_srvs::srv::Empty::Response>) {
      std::lock_guard<std::mutex> lock(mutex_);
      execution_timer_->cancel();
      execution_step_ = -1;
      is_moving_ = false;
      current_state_ = ArmState::ERROR;
    });

  status_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto status_msg = std_msgs::msg::String();
    switch (current_state_)
    {
      case ArmState::IDLE: status_msg.data = "IDLE"; break;
      case ArmState::RECEIVING: status_msg.data = "RECEIVING"; break;
      case ArmState::PLANNING: status_msg.data = "PLANNING"; break;
      case ArmState::MOVING: status_msg.data = "MOVING"; break;
      case ArmState::HOLDING: status_msg.data = "HOLDING"; break;
      case ArmState::COMPLETED: status_msg.data = "COMPLETED"; break;
      case ArmState::ERROR: status_msg.data = "ERROR"; break;
    }
    status_pub_->publish(status_msg);
  });

  execution_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
    std::bind(&ArmControllerNode::execution_callback, this));
  execution_timer_->cancel();
}

void ArmControllerNode::publish_joint_command(const std::string& joint_name, double position)
{
  auto it = joint_cmd_pubs_.find(joint_name);
  if (it == joint_cmd_pubs_.end()) {
    RCLCPP_WARN(this->get_logger(), "Unknown joint: %s", joint_name.c_str());
    return;
  }
  std_msgs::msg::Float64 msg;
  msg.data = position;
  it->second->publish(msg);
}

void ArmControllerNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  auto get_pos = [&](const char * name, double & out) {
    size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) {
      if (msg->name[i] == name) {
        out = msg->position[i];
        return;
      }
    }
  };

  get_pos("base_rotator", current_joint_positions_.base_rotator);
  get_pos("shoulder_pitch", current_joint_positions_.shoulder_pitch);
  get_pos("elbow_pitch", current_joint_positions_.elbow_pitch);
  get_pos("wrist_pitch", current_joint_positions_.wrist_pitch);
  get_pos("wrist_roll", current_joint_positions_.wrist_roll);
  get_pos("gripper_joint", current_joint_positions_.gripper);
}

CommandType ArmControllerNode::parse_command(const std::string& command)
{
  std::istringstream iss(command);
  std::string cmd;
  iss >> cmd;

  if (cmd == "PICK") return CommandType::PICK;
  if (cmd == "PLACE") return CommandType::PLACE;
  if (cmd == "HOLD") return CommandType::HOLD;
  if (cmd == "RELEASE") return CommandType::RELEASE;
  if (cmd == "HOME") return CommandType::HOME;
  if (cmd == "STOP") return CommandType::STOP;
  return CommandType::UNKNOWN;
}

std::vector<double> ArmControllerNode::parse_position(const std::string& command)
{
  std::vector<double> pos;
  std::istringstream iss(command);
  std::string word;

  iss >> word;

  int tokens = 0;
  while (iss >> word && pos.size() < 3 && tokens < 100)
  {
    tokens++;
    try {
      size_t idx;
      double val = std::stod(word, &idx);
      if (idx == word.length() && val >= -100.0 && val <= 100.0) {
        pos.push_back(val);
      }
    } catch (const std::invalid_argument&) {
    } catch (const std::out_of_range&) {
    }
  }
  return pos;
}

bool ArmControllerNode::validate_joint_position(const std::string& joint, double position) const
{
  static const std::map<std::string, std::pair<double, double>> limits = {
    {"base_rotator", {-3.14159, 3.14159}},
    {"shoulder_pitch", {-1.57, 1.57}},
    {"elbow_pitch", {-2.35, 2.35}},
    {"wrist_pitch", {-3.14, 3.14}},
    {"wrist_roll", {-3.14, 3.14}},
    {"gripper_joint", {0.0, 0.04}},
    {"gripper_joint_right", {0.0, 0.04}},
  };
  auto it = limits.find(joint);
  if (it != limits.end()) {
    if (position >= it->second.first && position <= it->second.second) {
      return true;
    }
    RCLCPP_WARN(this->get_logger(), "Joint %s position %f out of limits [%f, %f]",
      joint.c_str(), position, it->second.first, it->second.second);
    return false;
  }
  RCLCPP_WARN(this->get_logger(), "Unknown joint: %s", joint.c_str());
  return false;
}

void ArmControllerNode::start_async_execution(const JointPositions& target,
  GripperState gripper_action, ArmState completion_state, bool has_gripper_action)
{
  if (execution_step_ >= 0) {
    RCLCPP_WARN(this->get_logger(), "Already executing, rejecting command");
    return;
  }

  target_joint_positions_ = target;
  pending_gripper_state_ = gripper_action;
  pending_completion_state_ = completion_state;
  pending_has_gripper_action_ = has_gripper_action;
  execution_step_ = 0;
  execution_wait_ticks_ = 0;
  current_state_ = ArmState::PLANNING;
  execution_timer_->reset();
}

void ArmControllerNode::execution_callback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int ARM_WAIT_TICKS = 40;
  const int GRIPPER_WAIT_TICKS = 10;

  switch (execution_step_)
  {
    case 0:
    {
      current_state_ = ArmState::MOVING;
      is_moving_ = true;

      auto publish_validated = [this](const std::string& joint, double pos) {
        if (validate_joint_position(joint, pos)) {
          publish_joint_command(joint, pos);
        }
      };

      publish_validated("base_rotator", target_joint_positions_.base_rotator);
      publish_validated("shoulder_pitch", target_joint_positions_.shoulder_pitch);
      publish_validated("elbow_pitch", target_joint_positions_.elbow_pitch);
      publish_validated("wrist_pitch", target_joint_positions_.wrist_pitch);
      publish_validated("wrist_roll", target_joint_positions_.wrist_roll);

      execution_step_ = 1;
      execution_wait_ticks_ = 0;
      break;
    }
    case 1:
      execution_wait_ticks_++;
      if (execution_wait_ticks_ >= ARM_WAIT_TICKS) {
        if (pending_has_gripper_action_) {
          execution_step_ = 2;
          execution_wait_ticks_ = 0;
        } else {
          execution_step_ = 4;
        }
      }
      break;
    case 2:
    {
      double target_position = 0.04;
      switch (pending_gripper_state_)
      {
        case GripperState::OPEN: target_position = 0.04; break;
        case GripperState::CLOSED: target_position = 0.0; break;
        case GripperState::HOLDING: target_position = current_joint_positions_.gripper; break;
      }

      if (validate_joint_position("gripper_joint", target_position)) {
        publish_joint_command("gripper_joint", target_position);
        publish_joint_command("gripper_joint_right", target_position);
      }
      gripper_state_ = pending_gripper_state_;
      execution_step_ = 3;
      execution_wait_ticks_ = 0;
      break;
    }
    case 3:
      execution_wait_ticks_++;
      if (execution_wait_ticks_ >= GRIPPER_WAIT_TICKS) {
        execution_step_ = 4;
      }
      break;
    case 4:
      is_moving_ = false;
      current_state_ = pending_completion_state_;
      execution_step_ = -1;
      execution_timer_->cancel();
      break;
    default:
      break;
  }
}

void ArmControllerNode::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  RCLCPP_INFO(this->get_logger(), "Command received: %s", msg->data.c_str());

  rclcpp::Time now = this->now();
  if ((now - last_command_time_).seconds() < 0.1) {
    RCLCPP_WARN(this->get_logger(), "Rate limit exceeded, rejecting command");
    return;
  }
  last_command_time_ = now;

  if (execution_step_ >= 0) {
    RCLCPP_WARN(this->get_logger(), "Busy executing previous command, rejecting");
    return;
  }

  current_state_ = ArmState::RECEIVING;

  CommandType cmd = parse_command(msg->data);

  switch (cmd)
  {
    case CommandType::PICK:
    {
      auto pos = parse_position(msg->data);
      if (pos.size() >= 3)
      {
        JointPositions pick_pos;
        pick_pos.base_rotator = std::atan2(pos[1], pos[0]);
        pick_pos.shoulder_pitch = 0.5;
        pick_pos.elbow_pitch = 1.0;
        start_async_execution(pick_pos, GripperState::CLOSED, ArmState::HOLDING, true);
      }
      break;
    }
    case CommandType::PLACE:
    {
      auto pos = parse_position(msg->data);
      if (pos.size() >= 3)
      {
        JointPositions place_pos;
        place_pos.base_rotator = std::atan2(pos[1], pos[0]);
        place_pos.shoulder_pitch = 0.5;
        place_pos.elbow_pitch = 1.0;
        start_async_execution(place_pos, GripperState::OPEN, ArmState::IDLE, true);
      }
      break;
    }
    case CommandType::HOLD:
      start_async_execution(target_joint_positions_, GripperState::HOLDING, ArmState::HOLDING, true);
      break;
    case CommandType::RELEASE:
      start_async_execution(target_joint_positions_, GripperState::OPEN, ArmState::IDLE, true);
      break;
    case CommandType::HOME:
      start_async_execution(JointPositions(), GripperState::OPEN, ArmState::IDLE, false);
      break;
    case CommandType::STOP:
      execution_timer_->cancel();
      execution_step_ = -1;
      is_moving_ = false;
      current_state_ = ArmState::ERROR;
      break;
    default:
      current_state_ = ArmState::ERROR;
      break;
  }
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoboticArm::ArmControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
