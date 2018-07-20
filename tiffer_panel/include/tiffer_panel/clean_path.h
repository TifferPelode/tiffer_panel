///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 7/19/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit
//
////////////////////////////////////////////////////////////////////////////////

#ifndef CLEAN_PATH_H
#define CLEAN_PATH_H

#include <QDebug>
#include <QObject>
#include <QProcess>
#include <QThread>

#include "tiffer_panel/tiffer_panel.h"
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;
namespace Tiffer {
namespace Navigation {
class Clean_thread : public QThread {
  // Q_OBJECT

public:
  Clean_thread() {}

private:

  MoveBaseClient* ac;

  void setPose(move_base_msgs::MoveBaseGoal &goal, double p_x, double p_y, double theta) {
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.y = sin(theta/2);
    goal.target_pose.pose.orientation.w = cos(theta/2);

    ac->sendGoal(goal);
    ac->waitForResult();
  }

  void run() {

    ac = new MoveBaseClient("move_base", true);

    while (!ac->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server");
    }

    move_base_msgs::MoveBaseGoal goal;

    setPose(goal, 1.0, 0.0, 90.0);

    ROS_INFO("Sending goal 1");

    if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("You have arrived to the goal position");
    } else {
      ROS_INFO("The base failed for some reason");
    }
  }
};
} // namespace Navigation
} // namespace Tiffer

#endif // CLEAN_PATH_H