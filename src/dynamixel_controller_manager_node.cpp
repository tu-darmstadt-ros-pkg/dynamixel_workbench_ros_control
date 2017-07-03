#include <dynamixel_workbench_ros_control/dynamixel_hardware_interface.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_controller_manager");
  ROS_INFO_STREAM("Starting hardware interface");

  ros::NodeHandle pnh("~");
  ros::Rate rate(pnh.param("control_loop_hz", 25));

  dynamixel_workbench_ros_control::DynamixelHardwareInterface hw;
  if (!hw.init(pnh)) {
    ROS_ERROR_STREAM("Failed to initialize hardware interface.");
    return 1;
  }
  ROS_INFO_STREAM("Finished initializing HW interface");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  ros::AsyncSpinner spinner(1, &queue);
  spinner.start();
  controller_manager::ControllerManager cm(&hw, nh);

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
      //ROS_INFO_STREAM("Running controller update: Time: " << current_time << ", Period: " << period);
      cm.update(current_time, period);
    }
    hw.write();
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
