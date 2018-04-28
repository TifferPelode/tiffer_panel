///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 4/23/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit           
//
////////////////////////////////////////////////////////////////////////////////

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <rviz/display_context.h>

#include "tiffer_panel/tiffer_cruiseLocation.h"

namespace tiffer_panel
{
    CruiseLocationTool::CruiseLocationTool()
    {

    }

    void CruiseLocationTool::onInitialize()
    {
        PoseTool::onInitialize();
        //setName("Tiffer Cruise Location");
        updateTopic();
    }

    void CruiseLocationTool::updateTopic()
    {
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/Tiffer/MouseCruiseLocation", 1);
    }

    void CruiseLocationTool::onPoseSet(double x, double y, double theta)
    {
        bool ok;
        std::string name =QInputDialog::getText(0, QString::fromUtf8("添加新的巡航位置"), QString::fromUtf8("位置名称："),
                                                QLineEdit::Normal, "", &ok).toStdString();
        if(!ok)
            return;
        std::string fixed_frame = context_->getFixedFrame().toStdString();
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), name);
        geometry_msgs::PoseStamped goal;
        tf::poseStampedTFToMsg(p, goal);
        ROS_INFO(
            "Setting cruise location: Frame: %s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n",
            fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
            goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w,
            theta);
        pub_.publish(goal);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tiffer_panel::CruiseLocationTool, rviz::Tool)