#ifndef DYNAMIXEL_HARWARE_INTERFACE_H
#define DYNAMIXEL_HARWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>

namespace dynamixel_workbench_ros_control {

class DynamixelHardwareInterface : public hardware_interface::RobotHW
{
public:
  DynamixelHardwareInterface();

  bool init();
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

  std::vector<transmission_interface::SimpleTransmission> transmission_;
  std::vector<transmission_interface::ActuatorData> a_state_;
  std::vector<transmission_interface::ActuatorData> a_goal_;

  std::vector<transmission_interface::JointData> j_state_;
  std::vector<transmission_interface::JointData> j_goal_;


  std::vector<double> goal_position_;
  std::vector<double> goal_effort_;
  std::vector<double> goal_velocity_;

  std::vector<double> current_position_;
  std::vector<double> current_velocity_;
  std::vector<double> current_effort_;
};

}



#endif
