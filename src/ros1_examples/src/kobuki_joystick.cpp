#include "ros1_examples/kobuki_joystick.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/MotorPower.h"

#include <fcntl.h>
#include <unistd.h>

namespace kobuki_joystick
{
enum DS4_BUTTONS
{
  CROSS,
  CIRCLE,
  TRIANGLE,
  SQUARE,
  L1,
  R1,
};

enum DS4_AXIS
{
  L3_X,
  L3_Y,
  //L2, New versions of the driver only.
  R3_X,
  R3_Y,
};

bool KobukiJoystick::init()
{
  ros::NodeHandle nh("~");

  nh.param<std::string>("input_device", m_input_device, m_input_device);
  nh.param<float>("scale_linear", m_scale_linear, m_scale_linear);
  nh.param<float>("scale_angular", m_scale_angular, m_scale_angular);

  ROS_INFO_STREAM("KobukiJoystick : input device [" << m_input_device << "]");
  m_fd = open(m_input_device.c_str(), O_RDONLY | O_NONBLOCK);
  if (m_fd == -1)
  {
    ROS_ERROR_STREAM("KobukiJoystick: Error opening joystick device \""
                     << m_input_device
                     << "\", is the joystick paired and connected?");
    return false;
  }

  m_velocity_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  m_motor_power_publisher =
    nh.advertise<kobuki_msgs::MotorPower>("motor_power", 1);

  return true;
}

void KobukiJoystick::spin()
{
  ros::Rate loop_rate(60);

  js_event event;
  while (ros::ok())
  {
    if (readEvent(event))
    {
      if (event.type == JS_EVENT_BUTTON && event.number == DS4_BUTTONS::L1)
      {
        if (!m_enabled && event.value == 1)
        {
          enable();
          m_enabled = true;
        }
        else if (m_enabled && event.value == 0)
        {
          disable();
          m_enabled = false;
        }
      }
      else if (event.type == JS_EVENT_AXIS)
      {
        if (event.number == DS4_AXIS::L3_Y)
        {
          m_twist_msg->linear.x = -event.value / 32767.0 * m_scale_linear;
        }
        else if (event.number == DS4_AXIS::R3_X)
        {
          m_twist_msg->angular.z = -event.value / 32767.0 * m_scale_angular;
        }
      }
    }

    if (m_enabled && (m_twist_msg->linear.x != 0 || m_twist_msg->angular.z != 0))
    {
      m_velocity_publisher.publish(m_twist_msg);
    }

    loop_rate.sleep();
  }
}

bool KobukiJoystick::readEvent(js_event& event)
{
  if (read(m_fd, &event, sizeof(event)) == -1)
  {
      return false;
  }

  event.type &= ~JS_EVENT_INIT;

  return true;
}

void KobukiJoystick::enable()
{
  kobuki_msgs::MotorPower power_cmd;
  power_cmd.state = kobuki_msgs::MotorPower::ON;
  m_motor_power_publisher.publish(power_cmd);
}

void KobukiJoystick::disable()
{
  m_twist_msg->linear.x = 0;
  m_twist_msg->angular.z = 0;
  m_velocity_publisher.publish(m_twist_msg);
  kobuki_msgs::MotorPower power_cmd;
  power_cmd.state = kobuki_msgs::MotorPower::OFF;
  m_motor_power_publisher.publish(power_cmd);
}
}  // namespace kobuki_joystick

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "kobuki_joystick");

    kobuki_joystick::KobukiJoystick joystick;
    if (joystick.init())
    {
        joystick.spin();
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise KobukiJoystick!");
    }

    ROS_INFO_STREAM("Cave Johnson. We're Done Here");

    return 0;
}
