#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <rtabmap_ros/Info.h>
#include <nav_msgs/OccupancyGrid.h>
#include <RovyMotorController.h>

static ros::Timer twistTimer;
static RovyMotorController *controller = NULL;
static bool keyboardActive = false;
static bool mapReceived = false;
static ros::Subscriber rtabmap, map;
static ros::NodeHandle *nodeHande = NULL;

void twistTimerCallback(const ros::TimerEvent& event) {
    ROS_INFO("---- twist timer called ----");
    if (controller) {
        controller->drive(0, 0);
    }
    keyboardActive = false;
}

void plannerCallback(const geometry_msgs::TwistConstPtr& msg) {
    if (!keyboardActive) {
        ROS_INFO("Planner Linear: %f Angular: %f", msg->linear.x, msg->angular.z);

        if (controller) {
            controller->drive(msg->linear.x, msg->angular.z);
        }

        twistTimer.stop();
        twistTimer.start();
    }
}

void keyboardCallback(const geometry_msgs::TwistConstPtr& msg) {
    keyboardActive = true;

    if (controller) {
        controller->drive(msg->linear.x, msg->angular.z);
    }

    twistTimer.stop();
    twistTimer.start();
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    map.shutdown();
    mapReceived = true;
}

void rotateUntilLocalized() {
    bool rotate = true;
    int count = 0;
    ros::Rate rate(10);

    while ((access("/tmp/relocated", F_OK ) != -1 || !mapReceived) && ros::ok()) {
        if (!keyboardActive) {
            ROS_INFO_ONCE("rotating for localization...");
            if (count++ > 25) {
                rotate = !rotate;
                count = 0;
            }

            if (controller) {
                controller->drive(0, rotate ? 0.7 : 0);
            }
        }
        rate.sleep();
    }

    if (controller) {
        controller->drive(0, 0);
    }

    if (mapReceived) {
        ROS_INFO("localized!");
    }
}

void rtabmapCallback(const rtabmap_ros::Info& msg) {
    rtabmap.shutdown();
    ROS_INFO("rtabmap started");

    map = nodeHande->subscribe("map", 1, mapCallback);

    sleep(1);

    rotateUntilLocalized();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rovy_base_controller");
  ros::NodeHandle n;
  nodeHande = &n;

  ros::Subscriber plaSub = n.subscribe("cmd_vel", 1, plannerCallback);
  ros::Subscriber keySub = n.subscribe("key_vel", 1, keyboardCallback);

  // this callback is triggered when rtabmap is started
  rtabmap = n.subscribe("rtabmap/info", 1, rtabmapCallback);

  controller = RovyMotorController::Create();
  if (controller->start(550, 3) < 0) {
      ROS_ERROR("controller->start() returned error code");
      return -1;
  }

  twistTimer = n.createTimer(ros::Duration(0.5), twistTimerCallback, true);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  twistTimer.stop();
  controller->stop();
  rtabmap.shutdown();
  map.shutdown();

  return 0;
}
