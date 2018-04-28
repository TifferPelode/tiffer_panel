///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 4/20/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit           
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TIFFER_PANEL_H
#define TIFFER_PANEL_H

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <rviz/panel.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>

#include "tiffer_panel/tiffer_locationManager.h"

#endif

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>
#include <QProgressBar>
#include <QListWidget>
#include <QLabel>
#include <QTimer>
#include <QLine>

#include <QDebug>


namespace tiffer_panel {

class TifferPanel: public rviz::Panel
{
    Q_OBJECT
    public:
        TifferPanel( QWidget* parent = 0 );

        virtual void load( const rviz::Config& config );
        virtual void save( rviz::Config config ) const;

        void message_cb(std_msgs::String msg);
      

    public Q_SLOTS:
        void setMessage( const QString& message );
        void setTopic();

        void localizeCallback();
        void addLocationCallback();
        void removeLocationCallback();
        void navigationCallback();
        void stopCallback();

    private:
        void addLine(QVBoxLayout* layout);
        bool addLocation(const geometry_msgs::Pose &pose, const std::string &name);
        void getCurrentLocation(geometry_msgs::Pose &pose);

        ros::NodeHandle nh_;
        ros::Publisher location_mark_pub_;
        ros::Publisher nav_stop_pub_;
        LocationManagerPtr location_manager_;
        QComboBox* location_box_;
        QLineEdit* status_line_;

    protected:

        /// One-line text editor for entering the ROS topic to monitor for messages.
        QLineEdit* input_topic_editor;

        /// Where to display the status messages.
        QLabel* message_display;

        /// The current name of the input topic.
        QString input_topic;

        /// The ROS publisher for the incoming messages.
        ros::Subscriber subscriber;

        ros::NodeHandle nh;


    };

} // end namespace

#endif

