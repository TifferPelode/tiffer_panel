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
#include <QFile>

#include "tiffer_panel/tiffer_panel.h"
#include <std_msgs/Float64.h>
#include <cmath>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;
namespace Tiffer {
namespace Navigation {

struct Pose {
  double p_x_;
  double p_y_;
  double theta_;
};

class Clean_thread : public QThread {
  // Q_OBJECT

public:
  Clean_thread() {}

private:

  MoveBaseClient* mbc_;
  std::vector<Pose> v_pose_;
  ros::NodeHandle nh_;
  ros::Publisher clean_progress_pub_;
  

  void setPose(move_base_msgs::MoveBaseGoal &goal, double p_x, double p_y, double theta) {
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = p_x;
    goal.target_pose.pose.position.y = p_y;
    goal.target_pose.pose.orientation.z = sin(theta/2);
    goal.target_pose.pose.orientation.w = cos(theta/2);

    mbc_->sendGoal(goal);
    mbc_->waitForResult();
  }

  void run() {
    clean_progress_pub_ = nh_.advertise<std_msgs::Float64>("/clean_process", 1, true);
    
    std::string path_to_file = ros::package::getPath("tiffer_panel") + "/file/clean_path.txt";
    QFile file(QString::fromStdString(path_to_file));
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
      return ;

    QStringList str_list;
    QTextStream in(&file);
    Pose pp;
    while(!in.atEnd()) {
      QString line = in.readLine();
      str_list = line.simplified().split(" ");
      pp.p_x_ = str_list.at(0).toDouble();
      pp.p_y_ = str_list.at(1).toDouble();
      pp.theta_ = str_list.at(2).toDouble();
      v_pose_.push_back(pp);
    }

    for(std::vector<Pose>::iterator it = v_pose_.begin(); it != v_pose_.end(); ++it){
      qDebug() << it->p_x_ << it->p_y_ << it->theta_;
    }

    file.close();

    mbc_ = new MoveBaseClient("move_base", true);

    while (!mbc_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server");
    }

    move_base_msgs::MoveBaseGoal goal;

    std_msgs::Float64 clean_pro;
    clean_pro.data = 0.0;

    for(auto it = v_pose_.begin(); it != v_pose_.end(); ++it){
      setPose(goal, it->p_x_, it->p_y_, it->theta_);
      clean_pro.data = (it - v_pose_.begin()) / (1.0 * (v_pose_.size() - 1)) * 100.0;
      //clean_progress_->setValue(clean_pro);
      clean_progress_pub_.publish(clean_pro);
    }

    v_pose_.clear();

    if (mbc_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("You have arrived to the goal position");
    } else {
      ROS_INFO("The base failed for some reason");
    }
  }
};
} // namespace Navigation
} // namespace Tiffer

#endif // CLEAN_PATH_H