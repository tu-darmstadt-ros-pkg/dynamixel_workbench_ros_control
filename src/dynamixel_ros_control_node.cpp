#include <dynamixel_workbench_ros_control/dynamixel_hardware_interface.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_controller_manager");
  dynamixel_workbench_ros_control::DynamixelHardwareInterface hw;
  controller_manager::ControllerManager cm(&hw);

  ros::NodeHandle pnh("~");
  ros::Rate rate(pnh.param("control_loop_hz", 25));

  ros::Time current_time = ros::Time::now();
  bool first_update = true;
  while (ros::ok())
  {
    hw.read();
    ros::Duration period = ros::Time::now() - current_time;
    current_time = ros::Time::now();
    if (first_update) {
      first_update = false;
    } else {
      cm.update(current_time, period);
    }
    hw.write();
    rate.sleep();
  }
  return 0;
}
