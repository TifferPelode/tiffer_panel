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
#include <ros/package.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <rviz/panel.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>

#include "tiffer_panel/tiffer_locationManager.h"
#include "tiffer_panel/tiffer_record.h"
#include "tiffer_panel/clean_path.h"

#include <cmath>

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
#include <QProcess>
#include <QProgressBar>
#include <QListWidget>
#include <QLabel>
#include <QTimer>
#include <QLine>
#include <QSpacerItem>
#include <QAudioInput>
#include <QProgressBar>
#include <QRadioButton>
#include <QIcon>

#include <QDebug>

using namespace Tiffer::Navigation;

namespace Tiffer 
{
    namespace Navigation
    {
        enum class NavStatus : unsigned int
        {
            IDLE = 0,
            SUCCESS = 10,
            INPROGRESS = 20,
            FAILED = 30,
            SELF_LOCALIZATION = 40,
            WAIT_APPLICATION = 50
        };
    }

    namespace Visualization
    {
        class TifferPanel: public rviz::Panel
        {
            Q_OBJECT
            public:
                TifferPanel( QWidget* parent = 0 );
                virtual ~TifferPanel();

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
                void odomCallback(const nav_msgs::OdometryConstPtr &msg);
                void removeCruiseCallback();
                void goToLocationCallback();
                void startCruising();
                void clearCruise();
                void asrPressCallback();
                void asrReleaseCallback();
                void asrThreadCallback();

                void b1Callback();
                void b2Callback();
                void b3Callback();
                void b4Callback();
                void manualCallback();

            private:
                void addLine(QVBoxLayout* layout);
                void publishLocationsToRviz();
                bool addLocation(const geometry_msgs::Pose &pose, const std::string &name);
                void addToCruiseCallback(const KnownLocation &location);
                void getCurrentLocation(geometry_msgs::Pose &pose);
                void setRobotStatus(const NavStatus &status);
                void moveToLocation(const KnownLocation &location);
                void goToNextCruiseLocation();
                void mouseCruiseLocationCallback(const geometry_msgs::PoseStampedConstPtr &msg);
                void moveBaseResultCallback(const move_base_msgs::MoveBaseActionResultConstPtr &msg);
                void statusCallback(const move_base_msgs::MoveBaseActionResultConstPtr &msg);
                void globalPathLenCallback(const nav_msgs::Path &msg);
                void remainNavTimeCallback(const geometry_msgs::PoseStampedConstPtr &msg);
                void finishApplicationCallback(const std_msgs::BoolConstPtr &msg);
                void cleanProcessCallback(const std_msgs::Float64 &msg);
                

                ros::NodeHandle nh_;
                ros::Publisher location_mark_pub_;
                ros::Publisher nav_stop_pub_;
                ros::Publisher cruise_path_pub_;
                ros::Publisher cruise_number_pub_;
                ros::Publisher application_start_pub_;
                ros::Publisher application_finish_pub_;
                ros::Subscriber application_finish_sub_;
                ros::Subscriber clean_progress_sub_;
                ros::Subscriber odom_sub_;
                ros::Subscriber nav_status_sub_;
                ros::Subscriber application_finish_sub;
                ros::Subscriber mouse_cruise_location_sub_;
                ros::Subscriber path_len_sub_;
                ros::Subscriber nav_time_sub_;

                nav_msgs::Odometry odom_;
                LocationManagerPtr location_manager_;
                visualization_msgs::MarkerArray location_marks_;
                visualization_msgs::MarkerArray cruise_number_mark_;
                visualization_msgs::Marker cruise_path_mark_;
                std::vector<KnownLocation> cruise_path_;
                actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
                geometry_msgs::PoseStamped goal_;
                move_base_msgs::MoveBaseGoal mbgoal_;
                bool in_cruise_mode_ = false;
                int current_cruise_index_;

                Record_thread* record_thread_;
                Clean_thread* clean_thread_;

                QComboBox* location_box_;
                QLabel* path_len_label_;
                QLabel* asr_result_label_;
                QLabel* known_location_label_;
                QLabel* path_label_;
                QLabel* status_label_;
                QLabel* battery_label_;
                QLabel* auto_charge_label_;
                QLabel* hand_display_;
                QLabel* cruise_label_;

                QLineEdit* status_line_;
                QListWidget* location_widget_;

                QPushButton* cruise_remove_button_;
                QPushButton* cruise_open_button_;
                QPushButton* cruise_save_button_;
                QPushButton* asr_button_;
                QPushButton* localize_button_;
                QPushButton* add_location_button_;
                QPushButton* remove_location_button_;
                QPushButton* go_button_;
                QPushButton* manual_localize_button_1_;
                QPushButton* manual_localize_button_2_;
                QPushButton* manual_localize_button_3_;
                QPushButton* manual_localize_button_4_;
                QPushButton* stop_button_;
                QPushButton* main_manu_button_;
                QPushButton* cruise_button_;
                QPushButton* cruise_cleaar_button_;

                QProgressBar* clean_progress_;
                QFile* result_file_;
                QAudioInput* input;

                ros::Subscriber battery_sub_;
                QProgressBar *battery_bar_;
                QRadioButton *auto_charge_button_;
                QPushButton *manual_charge_button_;

            protected:

                /// One-line text editor for entering the ROS topic to monitor for messages.
                QLineEdit* input_topic_editor;

                /// Where to display the status messages.
                QLabel* message_display;

                /// The current name of the input topic.
                QString input_topic;

                /// The ROS publisher for the incoming messages.
                ros::Subscriber subscriber;

                //ros::NodeHandle nh;


        };
    }

} // end namespace

#endif

