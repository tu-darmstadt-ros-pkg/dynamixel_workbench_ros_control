#ifndef DYNAMIXEL_HARWARE_INTERFACE_H
#define DYNAMIXEL_HARWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>

namespace dynamixel_workbench_ros_control {

template<typename T>
std::string vecToString(const std::vector<T>& vec) {
  std::stringstream ss;
  ss << "[";
  for (unsigned int i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() -1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

struct State {
  State() : position(0), velocity(0), effort(0) {}
  double position;
  double velocity;
  double effort;
};

struct Joint {
  std::string name;
  State current;
  State goal;
};

class DynamixelHardwareInterface : public hardware_interface::RobotHW
{
public:
  DynamixelHardwareInterface();

  bool init(ros::NodeHandle& nh);
  void read();
  void write();

private:
  bool loadDynamixels(ros::NodeHandle& nh);
  void setTorque(bool enabled);

  bool first_cycle_;

  boost::shared_ptr<dynamixel_multi_driver::DynamixelMultiDriver> driver_;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;

  int joint_count_;

  std::vector<std::string> joint_names_;

  std::vector<double> goal_position_;
  std::vector<double> goal_effort_;
  std::vector<double> goal_velocity_;

  std::vector<double> current_position_;
  std::vector<double> current_velocity_;
  std::vector<double> current_effort_;
};

}



#endif
