///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 4/20/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit           
//
////////////////////////////////////////////////////////////////////////////////

#include "tiffer_panel/tiffer_panel.h"

namespace Tiffer 
{
    namespace Visualization
    {
        TifferPanel::TifferPanel( QWidget* parent ) :
        rviz::Panel( parent ),
        //input_topic("/status"),
        move_base_client_("/move_base", true),
        in_cruise_mode_(false)
        {

            QVBoxLayout* main_layout = new QVBoxLayout;

            QHBoxLayout* button_layout = new QHBoxLayout;

            // Next we lay out the "output topic" text entry field using a
            // QLabel and a QLineEdit in a QHBoxLayout.
            QHBoxLayout* topic_layout = new QHBoxLayout;
            topic_layout->addWidget( new QLabel( "Status Topic:" ));
            input_topic_editor = new QLineEdit;
            topic_layout->addWidget( input_topic_editor );

            QHBoxLayout* hand_layout = new QHBoxLayout;
            hand_display_ = new QLabel(QObject::trUtf8("功能选择"));
            hand_display_->setTextFormat(Qt::RichText);
            hand_display_->setAlignment(Qt::AlignCenter); 
            hand_layout->addWidget(hand_display_);
            main_layout->addStretch();
            main_layout->addLayout(hand_layout);

            QVBoxLayout* manual_layout = new QVBoxLayout;
            QHBoxLayout* v1_manual_layout = new QHBoxLayout;
            QHBoxLayout* v2_manual_layout = new QHBoxLayout;
            std::string c_path = ros::package::getPath("tiffer_panel") + "/icons/classes";
            std::string nav_icon_path = c_path + "compass.png";
            QIcon nav_icon(QString::fromStdString(nav_icon_path));
            manual_localize_button_1_ = new QPushButton(nav_icon, QObject::trUtf8("定点导航"));
            manual_localize_button_2_ = new QPushButton(nav_icon, QObject::trUtf8("语音控制"));
            manual_localize_button_3_ = new QPushButton(nav_icon, QObject::trUtf8("清扫状态"));
            manual_localize_button_4_ = new QPushButton(nav_icon, QObject::trUtf8("电量管理"));
            v1_manual_layout->addWidget(manual_localize_button_1_, 0, Qt::AlignVCenter);
            v1_manual_layout->addWidget(manual_localize_button_2_, 0, Qt::AlignVCenter);
            v2_manual_layout->addWidget(manual_localize_button_3_, 0, Qt::AlignVCenter);
            v2_manual_layout->addWidget(manual_localize_button_4_, 0, Qt::AlignVCenter);
            v1_manual_layout->addLayout(v1_manual_layout);
            v2_manual_layout->addLayout(v2_manual_layout);
            
            main_layout->addLayout(v1_manual_layout);
            main_layout->addLayout(v2_manual_layout);
            //addLine(main_layout);

            //Localize button
            localize_button_ = new QPushButton(QObject::trUtf8("自动定位"));
            localize_button_->hide();
            //localize_button->setFixedSize(200,100); //width height
            //main_layout->addWidget(localize_button, 0, Qt::AlignCenter);
            main_layout->addWidget(localize_button_);
            //addLine(main_layout);
            
            add_location_button_ = new QPushButton(QObject::trUtf8("添加当前位置"));
            add_location_button_->hide();
            button_layout->addWidget(add_location_button_);

            remove_location_button_ = new QPushButton(QObject::trUtf8("移除选中位置"));
            remove_location_button_->hide();
            button_layout->addWidget(remove_location_button_);

            main_layout->addLayout(button_layout);
            //addLine(main_layout);

            QVBoxLayout* known_location_layout = new QVBoxLayout;

            known_location_label_ = new QLabel(QObject::trUtf8("已知位置列表"));
            known_location_label_->hide();
            //known_location_label->setAlignment(Qt::AlignCenter);
            known_location_layout->addWidget(known_location_label_);

            location_box_ = new QComboBox;
            location_box_->hide();
            known_location_layout->addWidget(location_box_);
            main_layout->addLayout(known_location_layout);
            //addLine(main_layout);

            go_button_ = new QPushButton(QObject::trUtf8("导航至选中位置"));
            go_button_->hide();
            main_layout->addWidget(go_button_);
            stop_button_ = new QPushButton(QObject::trUtf8("停止"));
            stop_button_->hide();
            main_layout->addWidget(stop_button_);
            //addLine(main_layout);

            QHBoxLayout* cruise_button_layout = new QHBoxLayout;
            cruise_label_ = new QLabel(QObject::trUtf8("巡航路径"));
            cruise_label_->hide();
            cruise_button_layout->addWidget(cruise_label_);
            cruise_remove_button_ = new QPushButton(QObject::trUtf8("移除末端点"));
            cruise_remove_button_->hide();
            cruise_button_layout->addWidget(cruise_remove_button_);
            cruise_cleaar_button_ = new QPushButton(QObject::trUtf8("清除路经"));
            cruise_cleaar_button_->hide();
            cruise_button_layout->addWidget(cruise_cleaar_button_);
            main_layout->addLayout(cruise_button_layout);
            //addLine(main_layout);

            path_label_ = new QLabel();
            //path_label->setText(QObject::trUtf8("全局规划路经长度:"));
            path_label_->setText(QObject::trUtf8("导航剩余时间"));
            path_label_->hide();
            path_len_label_ = new QLabel();
            path_len_label_->hide();
            main_layout->addWidget(path_label_);
            main_layout->addWidget(path_len_label_);
            //addLine(main_layout);

            asr_button_ = new QPushButton(QObject::trUtf8("语音控制"));
            asr_button_->hide();
            main_layout->addWidget(asr_button_);
            asr_result_label_ = new QLabel();
            asr_result_label_->hide();
            main_layout->addWidget(asr_result_label_);
            //addLine(main_layout);

            location_widget_ = new QListWidget;
            location_widget_->hide();
            main_layout->addWidget(location_widget_);
            //addLine(main_layout);

            cruise_button_ = new QPushButton(QObject::trUtf8("开始巡航"));
            cruise_button_->hide();
            main_layout->addWidget(cruise_button_);
            //addLine(main_layout);

            clean_progress_ = new QProgressBar();
            clean_progress_->setOrientation(Qt::Horizontal);
            clean_progress_->setMinimum(0);
            clean_progress_->setMaximum(100);
            clean_progress_->setFormat(QString::fromLocal8Bit("当前清扫进度: %p%"));
            clean_progress_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            clean_progress_->hide();
            main_layout->addWidget(clean_progress_);
            //addLine(main_layout);

            QHBoxLayout* status_layout = new QHBoxLayout;
            status_label_ = new QLabel(QObject::trUtf8("状态"));
            status_label_->setStyleSheet("QLabel { font-size: 20px; }");
            status_layout->addWidget(status_label_);
            status_line_ = new QLineEdit();
            status_line_->setAlignment(Qt::AlignCenter);
            status_line_->setEnabled(false);
            status_layout->addWidget(status_line_);
            status_line_->setText(QObject::trUtf8("闲置"));
            status_line_->setStyleSheet(
                "QLineEdit { background: rgb(0, 255, 255); color: rgb(0, 0, 0); font-size: 20px;}");
            main_layout->addLayout(status_layout);
            main_layout->addStretch();
            status_label_->hide();
            status_line_->hide();
            //addLine(main_layout);

            //main_layout->addStretch();

            QHBoxLayout* battery_layout = new QHBoxLayout;
            battery_label_ = new QLabel(QObject::trUtf8("电池"));
            battery_label_->hide();
            battery_layout->addWidget(battery_label_);
            battery_bar_ = new QProgressBar;
            battery_bar_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            battery_bar_->hide();
            battery_layout->addWidget(battery_bar_);
            main_layout->addLayout(battery_layout);

            QHBoxLayout* charge_layout = new QHBoxLayout;
            auto_charge_label_ = new QLabel(QObject::trUtf8("自动充电"));
            auto_charge_label_->hide();
            charge_layout->addWidget(auto_charge_label_);
            auto_charge_button_ = new QRadioButton;
            auto_charge_button_->hide();
            charge_layout->addWidget(auto_charge_button_);
            manual_charge_button_ = new QPushButton(QObject::trUtf8("立刻充电"));
            manual_charge_button_->hide();
            charge_layout->addWidget(manual_charge_button_);
            manual_charge_button_->setEnabled(false);
            auto_charge_button_->setChecked(true);
            main_layout->addLayout(charge_layout);

            main_manu_button_ = new QPushButton(QObject::trUtf8("返回主菜单"));
            main_layout->addWidget(main_manu_button_);

            QHBoxLayout* message_layout = new QHBoxLayout;
            message_display = new QLabel(QObject::trUtf8("智澜科技"));
            message_display->setTextFormat(Qt::RichText);
            message_display->setAlignment(Qt::AlignCenter); 
            message_layout->addWidget(message_display);
            main_layout->addLayout(message_layout);

            setLayout( main_layout );

            record_thread_ = new Record_thread();
            clean_thread_ = new Clean_thread();

            //input_topic_editor->resize(150, input_topic_editor->height());

            // Next we make signal/slot connections.
            //connect( input_topic_editor, SIGNAL( editingFinished() ), this, SLOT( setTopic() ));
            connect(localize_button_, SIGNAL(clicked()), this, SLOT(localizeCallback()));
            connect(add_location_button_, SIGNAL(clicked()), this, SLOT(addLocationCallback()));
            connect(remove_location_button_, SIGNAL(clicked()), this, SLOT(removeLocationCallback()));
            connect(go_button_, SIGNAL(clicked()), this, SLOT(goToLocationCallback()));
            connect(stop_button_, SIGNAL(clicked()), this, SLOT(stopCallback()));
            //connect(cruise_remove_button_, SIGNAL(clicked()), this, SLOT(removeCruiseCallback()));
            //connect(cruise_cleaar_button, SIGNAL(clicked()), this, SLOT(clearCruise()));
            connect(cruise_button_, SIGNAL(clicked()), this, SLOT(startCruising()));
            //connect(asr_button_, SIGNAL(pressed()), this, SLOT(asrPressCallback()));
            connect(asr_button_, SIGNAL(released()), this, SLOT(asrReleaseCallback()));
            connect(record_thread_, SIGNAL(finished()), this, SLOT(asrThreadCallback()));

            connect(manual_localize_button_1_, SIGNAL(clicked()), this, SLOT(b1Callback()));
            connect(manual_localize_button_2_, SIGNAL(clicked()), this, SLOT(b2Callback()));
            connect(manual_localize_button_3_, SIGNAL(clicked()), this, SLOT(b3Callback()));
            connect(manual_localize_button_4_, SIGNAL(clicked()), this, SLOT(b4Callback()));
            connect(main_manu_button_, SIGNAL(clicked()), this, SLOT(manualCallback()));

            //input_topic_editor->setText( input_topic );
            //setTopic();

            location_manager_.reset(new LocationManager);

            for(auto&it : location_manager_->getLocations())
            {
                location_box_->addItem(QString::fromStdString(it.first));
            }

            location_mark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tiffer_panel/Navigation/KnownLocations", 2, true);
            publishLocationsToRviz();
            
            cruise_number_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tiffer_panel/Navigation/CruisePathNumber",2, true);
            cruise_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/tiffer_panel/Navigation/CruisePath",2,true);
            nav_stop_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1, true);
            odom_sub_ = nh_.subscribe("/odom", 10, &TifferPanel::odomCallback, this);
            nav_status_sub_ = nh_.subscribe("/move_base/result", 10, &TifferPanel::moveBaseResultCallback, this);
            path_len_sub_ = nh_.subscribe("/move_base/NavfnROS/plan", 10, &TifferPanel::globalPathLenCallback, this);
            nav_time_sub_ = nh_.subscribe("/move_base/current_goal", 10, &TifferPanel::remainNavTimeCallback, this);
            mouse_cruise_location_sub_ = nh_.subscribe("/tiffer_panel/MouseCruiseLocation", 10, &TifferPanel::mouseCruiseLocationCallback, this);
            
            cruise_path_mark_.type = cruise_path_mark_.LINE_STRIP;
            cruise_path_mark_.action = cruise_path_mark_.ADD;
            cruise_path_mark_.header.frame_id = "map";
            cruise_path_mark_.points.clear();
            cruise_path_mark_.pose.orientation.w = 1;
            cruise_path_mark_.color.g = cruise_path_mark_.color.r = cruise_path_mark_.color.a = 1;
            cruise_path_mark_.scale.x = cruise_path_mark_.scale.y = cruise_path_mark_.scale.z = 0.05;
            cruise_number_mark_.markers.clear();
            cruise_path_.clear();

            application_start_pub_ = nh_.advertise<std_msgs::Bool>("/start_application", 2, false);
            application_finish_sub_ = nh_.subscribe("/finish_application", 10, &TifferPanel::finishApplicationCallback, this);
            application_finish_pub_ = nh_.advertise<std_msgs::Bool>("/finish_application", 2, false);

            clean_progress_sub_ = nh_.subscribe("/clean_process", 1, &TifferPanel::cleanProcessCallback, this);

            stopCallback();     
        }

        TifferPanel::~TifferPanel()
        {
            record_thread_->exit(0);
            record_thread_->wait();
            record_thread_->deleteLater();
            clean_thread_->exit(0);
            clean_thread_->wait();
            clean_thread_->deleteLater();
        }

        template<class T>
        int arr_len(T& arr)
        {
            return sizeof(arr) / sizeof(arr[0]);
        }

        void TifferPanel::b1Callback()
        {
            manual_localize_button_1_->hide();
            manual_localize_button_2_->hide();
            manual_localize_button_3_->hide();
            manual_localize_button_4_->hide();
            hand_display_->hide();

            localize_button_->show();
            add_location_button_->show();
            remove_location_button_->show();
            known_location_label_->show();
            location_box_->show();
            go_button_->show();
            stop_button_->show();
            path_label_->show();
            path_len_label_->show();

            asr_button_->hide();
            asr_result_label_->hide();

            cruise_label_->hide();
            cruise_remove_button_->hide();
            cruise_cleaar_button_->hide();
            location_widget_->hide();
            cruise_button_->hide();
            clean_progress_->show();
            status_label_->show();
            status_line_->show();

            battery_label_->hide();
            battery_bar_->hide();
            auto_charge_label_->hide();
            auto_charge_button_->hide();
            manual_charge_button_->hide();
        }

        void TifferPanel::b2Callback()
        {
            manual_localize_button_1_->hide();
            manual_localize_button_2_->hide();
            manual_localize_button_3_->hide();
            manual_localize_button_4_->hide();
            hand_display_->hide();

            localize_button_->hide();
            add_location_button_->hide();
            remove_location_button_->hide();
            known_location_label_->hide();
            location_box_->hide();
            go_button_->hide();
            stop_button_->hide();
            path_label_->hide();
            path_len_label_->hide();

            asr_button_->show();
            asr_result_label_->show();

            cruise_label_->hide();
            cruise_remove_button_->hide();
            cruise_cleaar_button_->hide();
            location_widget_->hide();
            cruise_button_->hide();
            clean_progress_->hide();
            status_label_->hide();
            status_line_->hide();

            battery_label_->hide();
            battery_bar_->hide();
            auto_charge_label_->hide();
            auto_charge_button_->hide();
            manual_charge_button_->hide();
        }
        void TifferPanel::b3Callback()
        {
            manual_localize_button_1_->hide();
            manual_localize_button_2_->hide();
            manual_localize_button_3_->hide();
            manual_localize_button_4_->hide();
            hand_display_->hide();

            localize_button_->hide();
            add_location_button_->hide();
            remove_location_button_->hide();
            known_location_label_->hide();
            location_box_->hide();
            go_button_->hide();
            stop_button_->hide();
            path_label_->hide();
            path_len_label_->hide();

            asr_button_->hide();
            asr_result_label_->hide();

            cruise_label_->show();
            cruise_remove_button_->show();
            cruise_cleaar_button_->show();
            location_widget_->show();
            cruise_button_->show();
            clean_progress_->show();
            status_label_->show();
            status_line_->show();

            battery_label_->hide();
            battery_bar_->hide();
            auto_charge_label_->hide();
            auto_charge_button_->hide();
            manual_charge_button_->hide();
        }
        void TifferPanel::b4Callback()
        {
            manual_localize_button_1_->hide();
            manual_localize_button_2_->hide();
            manual_localize_button_3_->hide();
            manual_localize_button_4_->hide();
            hand_display_->hide();

            localize_button_->hide();
            add_location_button_->hide();
            remove_location_button_->hide();
            known_location_label_->hide();
            location_box_->hide();
            go_button_->hide();
            stop_button_->hide();
            path_label_->hide();
            path_len_label_->hide();

            asr_button_->hide();
            asr_result_label_->hide();

            cruise_label_->hide();
            cruise_remove_button_->hide();
            cruise_cleaar_button_->hide();
            location_widget_->hide();
            cruise_button_->hide();
            clean_progress_->hide();
            status_label_->hide();
            status_line_->hide();

            battery_label_->show();
            battery_bar_->show();
            auto_charge_label_->show();
            auto_charge_button_->show();
            manual_charge_button_->show();
        }

        void TifferPanel::manualCallback()
        {
            manual_localize_button_1_->show();
            manual_localize_button_2_->show();
            manual_localize_button_3_->show();
            manual_localize_button_4_->show();
            hand_display_->show();

            localize_button_->hide();
            add_location_button_->hide();
            remove_location_button_->hide();
            known_location_label_->hide();
            location_box_->hide();
            go_button_->hide();
            stop_button_->hide();
            path_label_->hide();
            path_len_label_->hide();

            asr_button_->hide();
            asr_result_label_->hide();

            cruise_label_->hide();
            cruise_remove_button_->hide();
            cruise_cleaar_button_->hide();
            location_widget_->hide();
            cruise_button_->hide();
            clean_progress_->hide();
            status_label_->hide();
            status_line_->hide();

            battery_label_->hide();
            battery_bar_->hide();
            auto_charge_label_->hide();
            auto_charge_button_->hide();
            manual_charge_button_->hide();
        }

        void TifferPanel::setTopic()
        {
            if(subscriber) {
                subscriber.shutdown();
            }
            input_topic = input_topic_editor->text();
            //subscriber = nh.subscribe(std::string(input_topic.toStdString()), 100, &TifferPanel::message_cb, this);
            Q_EMIT configChanged();
        }

        // Save all configuration data from this panel to the given
        // Config object.  It is important here that you call save()
        // on the parent class so the class id and panel name get saved.
        void TifferPanel::save( rviz::Config config ) const
        {
            rviz::Panel::save( config );
            config.mapSetValue( "topic", input_topic );
        }

        // Load all configuration data for this panel from the given Config object.
        void TifferPanel::load( const rviz::Config& config )
        {
            rviz::Panel::load( config );
            QString topic;
            if( config.mapGetString( "topic", &topic ))
            {
                input_topic_editor->setText( topic );
                setTopic();
            }
        }

        void TifferPanel::setMessage( const QString& msg) {
            message_display->setText(QString("<span style='font-weight: bold; font-size: 14pt;'>") + msg + "</span>");
        }

        void TifferPanel::message_cb(std_msgs::String msg)
        {
            setMessage(QString(msg.data.c_str()));
        }

        void TifferPanel::localizeCallback()
        {
            QMessageBox confirmation(QMessageBox::Question, QObject::trUtf8("提示"), 
                QObject::trUtf8("一方风景"));
            QPushButton *yes = confirmation.addButton(trUtf8("确 认"), QMessageBox::YesRole);
            QPushButton *no  = confirmation.addButton(trUtf8("取 消"), QMessageBox::NoRole);

            confirmation.exec();

            if(confirmation.clickedButton() == yes){
                QMessageBox accept;
                setRobotStatus(NavStatus::INPROGRESS);
                clean_progress_->setMaximum(100);
                clean_progress_->setValue(0);

                clean_thread_->start();

                accept.setText(trUtf8("已确认desu"));
                accept.exec();
            }
            else if(confirmation.clickedButton() == no){
                QMessageBox reject;
                setRobotStatus(NavStatus::IDLE);
                reject.setText(trUtf8("已取消desu"));
                reject.exec();
            }
        }

        void TifferPanel::addLocationCallback()
        {
            geometry_msgs::Pose new_pose;
            getCurrentLocation(new_pose);
            bool is_ok;
            std::string name = 
                QInputDialog::getText(this, QObject::trUtf8("增加一个新的导航点"), 
                    QObject::trUtf8("名称:"), QLineEdit::Normal, "", &is_ok).toStdString();
            
            if(!is_ok)
                return ;
            addLocation(new_pose, name);
            qDebug() << "callback";
        }

        void TifferPanel::getCurrentLocation(geometry_msgs::Pose &pose)
        {
            tf::TransformListener listener;
            tf::StampedTransform transform;
            try
            {
                listener.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(2));
                listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            }
            catch(tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
                QMessageBox::information(this, "Error", QString::fromStdString(ex.what()));
                return ;
            }
            geometry_msgs::Transform tt;
            tf::transformTFToMsg(transform, tt);
            pose.position.x = tt.translation.x;
            pose.position.y = tt.translation.y;
            pose.position.z = tt.translation.z;
            pose.orientation.x = tt.rotation.x;
            pose.orientation.y = tt.rotation.y;
            pose.orientation.z = tt.rotation.z;
            pose.orientation.w = tt.rotation.w;
        }

        void TifferPanel::odomCallback(const nav_msgs::OdometryConstPtr &msg)
        {
            odom_ = *msg;
            //ROS_INFO_STREAM("current pose: \n    x: " << odom_.pose.pose.position.x);
            //ROS_INFO_STREAM("Goal odomCallback: \n    x: " << goal_.pose.position.x);
            double result;
            result = sqrt(pow((odom_.pose.pose.position.x - goal_.pose.position.x), 2) + pow((odom_.pose.pose.position.y - goal_.pose.position.y), 2));
            result /= odom_.twist.twist.linear.x;
            if(odom_.twist.twist.linear.x == 0 && odom_.twist.twist.angular.z == 0) {
                path_len_label_->setText(trUtf8("已到达目标点"));
                //ROS_INFO_STREAM("Get Goal");
            }
            else {
                path_len_label_->setNum(result);
                //ROS_INFO_STREAM("remain time :" << result);
            }
                
        }

        void TifferPanel::remainNavTimeCallback(const geometry_msgs::PoseStampedConstPtr &msg)
        {
            goal_ = *msg;
            //ROS_INFO_STREAM("Goal: \n    x: " << goal_.pose.position.x);
        }

        void TifferPanel::globalPathLenCallback(const nav_msgs::Path &msg)
        {
            int len_ = end(msg.poses) - begin(msg.poses);
            double sum_ = 0.0;
            for(auto i = 0; i < len_ - 1; i++)
            {
                sum_ += sqrt(pow((msg.poses[i+1].pose.position.x - msg.poses[i].pose.position.x), 2) + pow((msg.poses[i+1].pose.position.y - msg.poses[i].pose.position.y), 2));
            }
            qDebug() << sum_;
            qDebug() << "----------------";
            
            //path_len_label_->setNum(sum_);
        }

        bool TifferPanel::addLocation(const geometry_msgs::Pose &pose, const std::string &name)
        {
            if(location_manager_->knowLocation(name))
            {
                //QMessageBox::StandardButton replay;
                if(QMessageBox::question(this, QString::fromUtf8("确认"),
                        QString::fromStdString(name) + QString::fromUtf8(" 已经存在，点击确认更新。 "),
                            QMessageBox::Yes | QMessageBox::No) == QMessageBox::StandardButton::No)
                {
                    return false;
                }
            }
            else{
                location_box_->addItem(QString::fromStdString(name));
            }
            location_manager_->addLocation(name, pose);

            publishLocationsToRviz();
            return true;
        }

        void TifferPanel::publishLocationsToRviz()
        {
            location_marks_.markers.clear();
            for(auto&it : location_manager_->getLocations())
            {
                visualization_msgs::Marker new_mark;
                new_mark.header.frame_id = "map";
                new_mark.header.stamp = ros::Time::now();
                new_mark.ns = "TifferPelode";
                new_mark.id = location_marks_.markers.size();
                new_mark.action = new_mark.ADD;//visualization_msgs::Marker::ADD;
                new_mark.type = new_mark.TEXT_VIEW_FACING;
                new_mark.pose = it.second.location;
                new_mark.pose.position.z = 0.15;
                
                //it.first -> std::string
                // std::cout << "it -> first: " << it.first << std::endl;
                // QString tr1(tr(it.first.data()));

                // if(typeid(it.first) == typeid(std::string)) 
                //     std::cout << "it.first is std::string." << std::endl;

                // std::cout << (typeid(tr1) == typeid(QString)) << std::endl;

                // QString tr2 = tr1.toUtf8();
                // qDebug() << "tr2 is :" << tr2;
                // new_mark.text = tr2.toStdString();

                new_mark.text = it.first;  //text -> string

                new_mark.scale.z = 0.5;
                new_mark.color.r = new_mark.color.a = 1;
                location_marks_.markers.push_back(new_mark);
            }
            location_mark_pub_.publish(location_marks_);
        }

        void TifferPanel::removeLocationCallback()
        {
            std::string name = location_box_->currentText().toStdString();
            QMessageBox confirmation(QMessageBox::Question, QObject::trUtf8("确认"), 
                QObject::trUtf8("确认移除当前位置 [") + QString::fromStdString(name + "] ?"));
            QPushButton *yes = confirmation.addButton(trUtf8("确 认"), QMessageBox::YesRole);
            QPushButton *no  = confirmation.addButton(trUtf8("取 消"), QMessageBox::NoRole);

            confirmation.exec();

            if(confirmation.clickedButton() == yes){
                location_manager_->removeLocation(name);
                location_box_->removeItem(location_box_->currentIndex());
                publishLocationsToRviz();

                QMessageBox accept;
                setRobotStatus(NavStatus::INPROGRESS);
                accept.setText(trUtf8("已确认desu"));
                accept.exec();
            }
            else if(confirmation.clickedButton() == no){
                QMessageBox reject;
                setRobotStatus(NavStatus::IDLE);
                reject.setText(trUtf8("已取消desu"));
                reject.exec();
            }
        }

        void TifferPanel::goToLocationCallback()
        {
            std::string location_name = location_box_->currentText().toStdString();
            bool find_location = false;
            KnownLocation target_location;
            for(auto&it : location_manager_->getLocations())
            {
                if(location_name == it.first)
                {
                    find_location = true;
                    target_location = it.second;
                    break;
                }
            }
            if(!find_location)
            {
                //ERROR("Location " << location_name << " not found");
                qDebug() << "Location not found";
                return;
            }
            moveToLocation(target_location);
        }

        void TifferPanel::moveToLocation(const KnownLocation &location)
        {
            setRobotStatus(NavStatus::INPROGRESS);
            static int id = 0;
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.header.seq = id;
            id++;
            goal.target_pose.pose = location.location;

            if(!move_base_client_.waitForServer(ros::Duration(5.0))){
                //ERROR("Can not connect to move_base server");
                qDebug() << "Can not connect to move_base server";
            }
            else
                move_base_client_.sendGoal(goal);
        }

        void TifferPanel::mouseCruiseLocationCallback(const geometry_msgs::PoseStampedConstPtr &msg)
        {
            if(addLocation(msg->pose, msg->header.frame_id))
            {
                KnownLocation new_location;
                new_location.location = msg->pose;
                new_location.name = msg->header.frame_id;
                addToCruiseCallback(new_location);
            }
        }

        void TifferPanel::addToCruiseCallback(const KnownLocation &location)
        {
            if(cruise_path_.size() == 0)
            {
                KnownLocation current_location;
                getCurrentLocation(current_location.location);

                visualization_msgs::Marker new_mark;
                new_mark.header.frame_id = "map";
                new_mark.header.stamp = ros::Time::now();
                new_mark.id = cruise_number_mark_.markers.size();
                new_mark.action = new_mark.ADD;
                new_mark.type = new_mark.TEXT_VIEW_FACING;
                new_mark.pose = current_location.location;
                new_mark.pose.position.z = 0.5;
                new_mark.pose.orientation.w = 1;
                new_mark.text = std::to_string(cruise_number_mark_.markers.size());
                new_mark.scale.z = 0.5;
                new_mark.color.g = new_mark.color.b = new_mark.color.a = 1;
                cruise_number_mark_.markers.push_back(new_mark);
                geometry_msgs::Point pp;
                pp = current_location.location.position;
                pp.z = 0.2;
                cruise_path_mark_.points.push_back(pp);
            }
            cruise_path_.push_back(location);
            geometry_msgs::Point p;
            p = location.location.position;
            p.z = 0.2;
            cruise_path_mark_.points.push_back(p);

            visualization_msgs::Marker new_mark;
            new_mark.header.frame_id = "map";
            new_mark.header.stamp = ros::Time::now();
            new_mark.id = cruise_number_mark_.markers.size();
            new_mark.action = new_mark.ADD;
            new_mark.type = new_mark.TEXT_VIEW_FACING;
            new_mark.pose.position = p;
            new_mark.pose.position.z = 0.5;
            new_mark.pose.orientation.w = 1;
            new_mark.text = std::to_string(cruise_number_mark_.markers.size());
            new_mark.scale.z = 0.5;
            new_mark.color.g = new_mark.color.b = new_mark.color.a = 1;
            cruise_number_mark_.markers.push_back(new_mark);
            
            cruise_path_mark_.header.stamp = ros::Time::now();

            cruise_number_pub_.publish(cruise_number_mark_);
            cruise_path_pub_.publish(cruise_path_mark_);
            location_widget_->addItem(QString::fromStdString(location.name));
        }

        void TifferPanel::startCruising()
        {
            if(cruise_path_.size() == 0)
            {
                QMessageBox::information(this, "Error", QString::fromUtf8("没有找到巡航路经."));
                return;
            }
            for(int i = 0; i < cruise_path_.size(); i++)
            {
                location_widget_->item(i)->setText(
                    QString::fromStdString(cruise_path_[i].name) + QString::fromUtf8("  等待"));
                location_widget_->item(i)->setBackgroundColor(QColor::fromRgb(200, 200, 0, 255));
            }
            current_cruise_index_ = 0;
            in_cruise_mode_ = true;
            goToNextCruiseLocation();
        }

        void TifferPanel::statusCallback(const move_base_msgs::MoveBaseActionResultConstPtr &msg)
        {
            ROS_INFO_STREAM("Result" << msg->status);
            if(msg->status.status == msg->status.SUCCEEDED)
            {
                ROS_INFO_STREAM("current goal name: " << msg->status.text);
                ROS_INFO_STREAM("current status: " << msg->status);
                setRobotStatus(NavStatus::SUCCESS);
            }

            /*if(!in_cruise_mode_){
                setRobotStatus(NavStatus::IDLE);
            }
            else if(msg->status.status == msg->status.SUCCEEDED){
                std_msgs::Bool start_msg;
                start_msg.data = true;
            }*/
        }

        void TifferPanel::moveBaseResultCallback(const move_base_msgs::MoveBaseActionResultConstPtr &msg)
        {
            ROS_INFO_STREAM("Result "<<msg->status);
            if (msg->status.status == msg->status.SUCCEEDED)
            {
                ROS_INFO_STREAM("current goal name:"<<msg->status.text);
                ROS_INFO_STREAM("current status :"<<msg->status);
            }
            if (!in_cruise_mode_)
            {
                setRobotStatus(NavStatus::IDLE);
            }
            else if (msg->status.status == msg->status.SUCCEEDED)
            {
                std_msgs::Bool start_msg;
		        start_msg.data = true;
                application_start_pub_.publish(start_msg);
                setRobotStatus(NavStatus::WAIT_APPLICATION);

                if (current_cruise_index_ > 0)
                {
                    location_widget_->item(current_cruise_index_ - 1)->setText(
                            QString::fromStdString(cruise_path_[current_cruise_index_ - 1].name)
                                    + QString::fromUtf8("    已抵达"));
                    location_widget_->item(current_cruise_index_ - 1)->setBackgroundColor(
                            QColor::fromRgb(200, 200, 200, 255));
                    
                    std_msgs::Bool finish_msg;
		            finish_msg.data = true;
                    application_finish_pub_.publish(finish_msg);
                }
            }
            else
            {
                setRobotStatus(NavStatus::FAILED);
                ROS_ERROR_STREAM("Failed to cruise to location "<<cruise_path_[current_cruise_index_-1].name);
            }
        }

        void TifferPanel::finishApplicationCallback(const std_msgs::BoolConstPtr &msg)
        {
            if (current_cruise_index_ < cruise_path_.size())
            {
                //system("rosservice call /move_base/clear_costmaps");
                ROS_INFO_STREAM("costmaps clear ");
                goToNextCruiseLocation();
            }
            else
            {
                location_widget_->item(current_cruise_index_ - 1)->setText(
                        QString::fromStdString(cruise_path_[current_cruise_index_ - 1].name)
                                + QString::fromUtf8("    已抵达"));
                location_widget_->item(current_cruise_index_ - 1)->setBackgroundColor(
                        QColor::fromRgb(200, 200, 200, 255));
                ROS_INFO_STREAM("Navigation: Cruising task completed.");
                setRobotStatus(NavStatus::SUCCESS);
                //clearCruise();
                in_cruise_mode_ = false;
            }
        }

        void TifferPanel::goToNextCruiseLocation()
        {
            location_widget_->item(current_cruise_index_)->setText(
                    QString::fromStdString(cruise_path_[current_cruise_index_].name) + QString::fromUtf8("    巡航中"));
            location_widget_->item(current_cruise_index_)->setBackgroundColor(QColor::fromRgb(0, 255, 0, 255));
            moveToLocation(cruise_path_[current_cruise_index_]);
            current_cruise_index_++;
        }

        void TifferPanel::removeCruiseCallback()
        {
            if(cruise_path_mark_.points.size() > 0)
            {
                cruise_number_mark_.markers[cruise_number_mark_.markers.size() - 1].action = 
                    cruise_number_mark_.markers[cruise_number_mark_.markers.size() - 1].DELETE;
                cruise_path_mark_.points.pop_back();
                cruise_number_pub_.publish(cruise_number_mark_);
                cruise_path_pub_.publish(cruise_path_mark_);
                cruise_number_mark_.markers.pop_back();
                if(cruise_path_.size() > 0){
                    location_widget_->removeItemWidget(location_widget_->item(cruise_path_.size() - 1));
                    cruise_path_.pop_back();
                }
            }
        }

        void TifferPanel::stopCallback()
        {
            actionlib_msgs::GoalID stop_msg;
            nav_stop_pub_.publish(stop_msg);
            setRobotStatus(NavStatus::IDLE);
            //if(in_cruise_mode_)
                //this->clearCruise();    //will crush, with bug here
        }

        void TifferPanel::clearCruise()
        {
            for(int i = 0; i < cruise_number_mark_.markers.size(); i++)
                cruise_number_mark_.markers[i].action = cruise_number_mark_.markers[i].DELETE;
            cruise_number_pub_.publish(cruise_number_mark_);
            cruise_path_mark_.points.clear();
            cruise_path_pub_.publish(cruise_path_mark_);
            cruise_number_mark_.markers.clear();
            if(cruise_path_.size() > 0)
                cruise_path_.pop_back();
            cruise_path_.clear();
            location_widget_->clear();
            in_cruise_mode_ = false;
        }

        void TifferPanel::asrPressCallback()
        {
            qDebug() << "press";

            result_file_ =new QFile();

            std::string path_to_file = ros::package::getPath("tiffer_panel") + "/file/t.pcm";
            QString trans = QString::fromUtf8(path_to_file.c_str());
            result_file_->setFileName(trans);//设置其实设置音频文件的存放路径(输入音频名及存放路径)

            bool is_open =result_file_->open(QIODevice::WriteOnly | QIODevice::Truncate);

            if(!is_open)
            {
                qDebug()<<"打开失败失败"<<endl;
                exit(1);
            }

            QAudioFormat format;
            format.setSampleRate(16000); //设置采样的赫兹
            format.setChannelCount(1); //设置通道数
            format.setSampleSize(16);   //设置样本大小，一般为8或者16
            format.setCodec("audio/pcm");//设置编解码器
            format.setByteOrder(QAudioFormat::LittleEndian);//设置大小端
            format.setSampleType(QAudioFormat::UnSignedInt);//设置采样格式

            QAudioDeviceInfo info = QAudioDeviceInfo::defaultInputDevice();
            QString str=info.deviceName();

            qDebug()<<"使用的录音设备是:"<<str<<endl;

            if(!info.isFormatSupported(format))
            {
                //format = info.nearestFormat(format);
            }

            //input = new QAudioInput(format, this);

            qDebug() << "Record Start...";

            //input->start(result_file_);
        }

        void TifferPanel::asrReleaseCallback()
        {
            qDebug() << "release";

            asr_button_->setEnabled(false);

            /*if(0 == system(rec_com.data())){
                qDebug() << "record success.";
            }else{
                qDebug() << "record error.";
            }

            if(0 == system(asr_com.data())){
                qDebug() << "asr_bd_ol success.";
            }else{
                qDebug() << "asr_bd_ol failed.";
            }*/

            record_thread_->start();

        }

        void TifferPanel::asrThreadCallback()
        {
            std::string cur_path = ros::package::getPath("tiffer_panel");
            std::string file_com = cur_path + "/file/result";

            QFile res_file(file_com.data());
            if(res_file.exists()){
                qDebug() << "file exists.";
            }else{
                qDebug() << "file connot found.";
            }

            if(!res_file.open(QIODevice::ReadWrite)){
                qDebug() << "open failed.";
            }else{
                qDebug() << "open success.";
            }

            char buf[1024];
            qint64 len = res_file.readLine(buf, 1024);
            if(len != -1){
                qDebug() << buf;
            }
            asr_result_label_->setText(buf);

            std::string asr_res(buf);
            std::cout << asr_res << std::endl;
            bool get_pose = false;
            KnownLocation target_location;
            for(auto&it : location_manager_->getLocations())
            {
                //if(asr_res == it.first)
                if(std::string::npos != asr_res.find(it.first))
                {
                    get_pose = true;
                    target_location = it.second;
                    break;
                }
            }
            if(!get_pose)
            {
                qDebug() << "ASR_Location Not Found.";
            }
            moveToLocation(target_location);

            /*if(std::string::npos != str_match.find("c点")){
                qDebug() << "ok";
            }else{
                qDebug() << "not found";
            }*/

            res_file.close();

            asr_button_->setEnabled(true);

        }

        void TifferPanel::cleanProcessCallback(const std_msgs::Float64 &msg)
        {
            //std_msgs::Float64 tr = msg;
            clean_progress_->setValue(msg.data);
        }

        void TifferPanel::addLine(QVBoxLayout* layout)
        {
            QFrame *line = new QFrame;
            line->setFrameShape(QFrame::HLine);
            line->setFrameShadow(QFrame::Sunken);
            line->setLineWidth(1);
            layout->addWidget(line);
        }

        void TifferPanel::setRobotStatus(const NavStatus &status)
        {
            switch (status)
            {
                case NavStatus::IDLE:
                    status_line_->setText(QObject::trUtf8("闲置状态"));
                    status_line_->setStyleSheet(
                        "QLineEdit { background: rgb(0, 255, 255);"\
                        " color: rgb(0, 0, 0); font-size: 20px;}"
                    );
                    break;
                    
                case NavStatus::INPROGRESS:
                    status_line_->setText(QObject::trUtf8("导航中..."));
                    status_line_->setStyleSheet(
                        "QLineEdit { background: rgb(255, 0, 255);"\
                        " color: rgb(0, 0, 0); font-size: 20px;}"
                    );
                    break;

                case NavStatus::SUCCESS:
                    status_line_->setText(QObject::trUtf8("导航完成"));
                    status_line_->setStyleSheet(
                        "QLineEdit { background: rgb(255, 215, 0);"\
                        " color: rgb(0, 0, 0); font-size: 20px;}"
                    );
                    break;
                
                case NavStatus::FAILED:
                    status_line_->setText(QObject::trUtf8("失败了"));
                    status_line_->setStyleSheet(
                        "QLineEdit { background: rgb(255, 0, 0);"\
                        " color: rgb(0, 0, 0); font-size: 20px;}"
                    );
                    break;
                
                case NavStatus::SELF_LOCALIZATION:
                    status_line_->setText(QObject::trUtf8("自动定位中"));
                    status_line_->setStyleSheet(
                        "QLineEdit { background: rgb(100, 100, 100);"\
                        " color: rgb(0, 0, 0); font-size: 20px;}"
                    );
                    break;

                case NavStatus::WAIT_APPLICATION:
                    status_line_->setText(QObject::trUtf8("等待中"));
                    status_line_->setStyleSheet(
                        "QLineEdit { background: rgb(100, 100, 100);"\
                        " color: rgb(0, 0, 0); font-size: 20px;}"
                    );
                    break;
                
                default:
                    status_line_->setText(QObject::trUtf8("唔知啊"));
                    status_line_->setStyleSheet(
                        "QlineEdit { background: rgb(255, 0, 0);"\
                        " color: rgb(255, 0, 0); font-size: 20px;}"
                    );
            }
        }
    }

} // end namespace

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Tiffer::Visualization::TifferPanel, rviz::Panel )
