#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <RovyMotorController.h>

static ros::Timer timer;
static RovyMotorController *controller = NULL;
static bool keyboardActive = false;

void timerCallback(const ros::TimerEvent& event) {
    ROS_INFO("---- timer called ----");
    if (controller) {
        controller->drive(0, 0);
    }
    keyboardActive = false;
}

void plannerCallback(const geometry_msgs::TwistConstPtr& msg) {
    if (!keyboardActive) {
        ROS_INFO("Planner Linear: %f Angular: %f", msg->linear.x, msg->angular.z);

        if (controller) {
            controller->drive(msg->linear.x, msg->angular.z / (2*M_PI/360));
        }
    }
}

void keyboardCallback(const geometry_msgs::TwistConstPtr& msg) {
    keyboardActive = true;

//    ROS_INFO("Key Linear: %f Angular: %f", msg->linear.x, msg->angular.z);

    if (controller) {
        controller->drive(msg->linear.x, msg->angular.z);
    }

    timer.stop();
    timer.start();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rovy_base_controller");
  ros::NodeHandle n;

  ros::Subscriber plaSub = n.subscribe("cmd_vel", 1, plannerCallback);
  ros::Subscriber keySub = n.subscribe("key_vel", 1, keyboardCallback);

  controller = RovyMotorController::Create();
  if (controller->start(350) < 0) {
      ROS_ERROR("controller->start() returned error code");
      return -1;
  }

  timer = n.createTimer(ros::Duration(0.5), timerCallback, true);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  timer.stop();
  controller->stop();

  return 0;
}
