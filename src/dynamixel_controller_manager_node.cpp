#include <dynamixel_workbench_ros_control/dynamixel_hardware_interface.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_controller_manager");
  dynamixel_workbench_ros_control::DynamixelHardwareInterface hw;
  if (!hw.init()) {
    ROS_ERROR_STREAM("Failed to initialize hardware interface.");
    return 1;
  }
  controller_manager::ControllerManager cm(&hw);

  ros::NodeHandle pnh("~");
  ros::Rate rate(pnh.param("control_loop_hz", 25));

  ros::Time current_time = ros::Time::now();
  bool first_update = true;
  ROS_INFO_STREAM("Starting control loop with cycle time of " << rate.cycleTime() << " s.");
  while (ros::ok())
  {
    hw.read();
    ros::Duration period = ros::Time::now() - current_time;
    current_time = ros::Time::now();
    if (first_update) {
      first_update = false;
    } else {
      ROS_INFO_STREAM("Running controller update: Time: " << current_time << ", Period: " << period);
      cm.update(current_time, period);
    }
    hw.write();
    rate.sleep();
  }
  return 0;
}
