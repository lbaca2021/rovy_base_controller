#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <rtabmap_ros/Info.h>
#include <nav_msgs/OccupancyGrid.h>
#include <RovyMotorController.h>

static ros::Timer twistTimer;
static RovyMotorController *controller = NULL;
static bool keyboardActive = false;
static bool localized = false;
static ros::Subscriber rtabmap, proximity;
static ros::NodeHandle *nodeHande = NULL;

#define RELOCATION_FLAG_FILE "/tmp/relocated"

void rotateUntilLocalized() {
    if (!keyboardActive) {
        ROS_INFO("rotating for localization...");
    }

    bool rotate = true;
    int count = 0;
    ros::Rate rate(10);

    while (!localized && ros::ok()) {
        if (!keyboardActive) {
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

    if (localized) {
        ROS_INFO("localized!");
    }
}

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

void proximityCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    proximity.shutdown();
    ROS_INFO("proximity localization");
    localized = true;
}

void rtabmapCallback2(const rtabmap_ros::Info& msg) {
    if (msg.loopClosureId > 0) {
        ROS_INFO("loopClosureId localization");
        rtabmap.shutdown();
        remove(RELOCATION_FLAG_FILE);
        localized = true;
    }
}

void rtabmapCallback1(const rtabmap_ros::Info& msg) {
    rtabmap.shutdown();
    ROS_INFO("rtabmap started");

    if (msg.loopClosureId > 0) {
        ROS_INFO("localized!");
        return;
    }

    if (access(RELOCATION_FLAG_FILE, F_OK ) != -1) {
        // if file exists then we have been relocated and can't rely on proximity
        rtabmap = nodeHande->subscribe("rtabmap/info", 1, rtabmapCallback2);
    } else {
        // if file doesn't exist then we can use proximity
        proximity = nodeHande->subscribe("map", 1, proximityCallback);
    }

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
  rtabmap = n.subscribe("rtabmap/info", 1, rtabmapCallback1);

  controller = RovyMotorController::Create();
  if (controller->start(350, 3) < 0) {
      ROS_ERROR("controller->start() returned error code");
      return -1;
  }

  twistTimer = n.createTimer(ros::Duration(0.5), twistTimerCallback, true);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  twistTimer.stop();
  controller->stop();
  rtabmap.shutdown();
  proximity.shutdown();

  return 0;
}
