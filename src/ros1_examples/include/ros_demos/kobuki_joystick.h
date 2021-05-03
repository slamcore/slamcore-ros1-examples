#ifndef KOBUKI_JOYSTICK_H_
#define KOBUKI_JOYSTICK_H_

#include <linux/joystick.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace kobuki_joystick
{
class KobukiJoystick
{
public:
  KobukiJoystick() : m_twist_msg(new geometry_msgs::Twist) {}
  bool init();
  void spin();

private:
  bool readEvent(js_event& event);
  void enable();
  void disable();

private:
  std::string m_input_device{"/dev/input/js0"};
  float m_scale_linear{1.0f};
  float m_scale_angular{1.0f};
  int m_fd{0};
  bool m_enabled{false};
  ros::Publisher m_velocity_publisher;
  ros::Publisher m_motor_power_publisher;
  geometry_msgs::TwistPtr m_twist_msg;
};

}  // namespace kobuki_joystick

#endif
