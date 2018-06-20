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
        move_base_client_("/move_base", true)
        {

            QVBoxLayout* main_layout = new QVBoxLayout;

            QHBoxLayout* button_layout = new QHBoxLayout;

            // Next we lay out the "output topic" text entry field using a
            // QLabel and a QLineEdit in a QHBoxLayout.
            QHBoxLayout* topic_layout = new QHBoxLayout;
            topic_layout->addWidget( new QLabel( "Status Topic:" ));
            input_topic_editor = new QLineEdit;
            topic_layout->addWidget( input_topic_editor );

            //Localize button
            QPushButton* localize_button = new QPushButton(QObject::trUtf8("自动定位"));
            //localize_button->setFixedSize(200,100); //width height
            //main_layout->addWidget(localize_button, 0, Qt::AlignCenter);
            main_layout->addWidget(localize_button);
            addLine(main_layout);
            
            QPushButton* add_location_button = new QPushButton(QObject::trUtf8("添加当前位置"));
            button_layout->addWidget(add_location_button);

            QPushButton* remove_location_button = new QPushButton(QObject::trUtf8("移除选中位置"));
            button_layout->addWidget(remove_location_button);

            main_layout->addLayout(button_layout);
            addLine(main_layout);

            QVBoxLayout* known_location_layout = new QVBoxLayout;

            QLabel* known_location_label = new QLabel(QObject::trUtf8("已知位置列表"));
            //known_location_label->setAlignment(Qt::AlignCenter);
            known_location_layout->addWidget(known_location_label);

            location_box_ = new QComboBox;
            known_location_layout->addWidget(location_box_);
            main_layout->addLayout(known_location_layout);
            addLine(main_layout);

            QPushButton* go_button = new QPushButton(QObject::trUtf8("导航至选中位置"));
            main_layout->addWidget(go_button);
            QPushButton* stop_button = new QPushButton(QObject::trUtf8("停止"));
            main_layout->addWidget(stop_button);
            addLine(main_layout);

            QHBoxLayout* cruise_button_layout = new QHBoxLayout;
            QLabel* cruise_label = new QLabel(QObject::trUtf8("巡航路径"));
            cruise_button_layout->addWidget(cruise_label);
            cruise_remove_button_ = new QPushButton(QObject::trUtf8("移除末端点"));
            cruise_button_layout->addWidget(cruise_remove_button_);
            QPushButton* cruise_cleaar_button = new QPushButton(QObject::trUtf8("清除路经"));
            cruise_button_layout->addWidget(cruise_cleaar_button);
            main_layout->addLayout(cruise_button_layout);
            addLine(main_layout);

            QLabel* path_label = new QLabel();
            //path_label->setText(QObject::trUtf8("全局规划路经长度:"));
            path_label->setText(QObject::trUtf8("导航剩余时间"));
            path_len_label_ = new QLabel();
            main_layout->addWidget(path_label);
            main_layout->addWidget(path_len_label_);
            addLine(main_layout);

            QPushButton* asr_button = new QPushButton(QObject::trUtf8("语音"));
            main_layout->addWidget(asr_button);
            asr_result_label_ = new QLabel();
            main_layout->addWidget(asr_result_label_);
            addLine(main_layout);

            location_widget_ = new QListWidget;
            main_layout->addWidget(location_widget_);
            addLine(main_layout);

            QPushButton* cruise_button = new QPushButton(QObject::trUtf8("开始巡航"));
            main_layout->addWidget(cruise_button);
            addLine(main_layout);

            QHBoxLayout* status_layout = new QHBoxLayout;
            QLabel* status_label = new QLabel(QObject::trUtf8("状态"));
            status_label->setStyleSheet("QLabel { font-size: 20px; }");
            status_layout->addWidget(status_label);
            status_line_ = new QLineEdit();
            status_line_->setAlignment(Qt::AlignCenter);
            status_line_->setEnabled(false);
            status_layout->addWidget(status_line_);
            status_line_->setText(QObject::trUtf8("闲置"));
            status_line_->setStyleSheet(
                "QLineEdit { background: rgb(0, 255, 255); color: rgb(0, 0, 0); font-size: 20px;}");
            main_layout->addLayout(status_layout);
            main_layout->addStretch();

            QHBoxLayout* message_layout = new QHBoxLayout;
            message_display = new QLabel(QObject::trUtf8("智澜科技"));
            message_display->setTextFormat(Qt::RichText);
            message_display->setAlignment(Qt::AlignCenter); 
            
            //main_layout->addWidget(message_display);
            message_layout->addWidget(message_display);
            main_layout->addLayout(message_layout);

            setLayout( main_layout );

            //input_topic_editor->resize(150, input_topic_editor->height());

            // Next we make signal/slot connections.
            //connect( input_topic_editor, SIGNAL( editingFinished() ), this, SLOT( setTopic() ));
            connect(localize_button, SIGNAL(clicked()), this, SLOT(localizeCallback()));
            connect(add_location_button, SIGNAL(clicked()), this, SLOT(addLocationCallback()));
            connect(remove_location_button, SIGNAL(clicked()), this, SLOT(removeLocationCallback()));
            connect(go_button, SIGNAL(clicked()), this, SLOT(goToLocationCallback()));
            connect(stop_button, SIGNAL(clicked()), this, SLOT(stopCallback()));
            connect(cruise_remove_button_, SIGNAL(clicked()), this, SLOT(removeCruiseCallback()));
            connect(cruise_cleaar_button, SIGNAL(clicked()), this, SLOT(clearCruise()));
            connect(cruise_button, SIGNAL(clicked()), this, SLOT(startCruising()));
            //connect(asr_button, SIGNAL(pressed()), this, SLOT(asrPressCallback()));
            connect(asr_button, SIGNAL(released()), this, SLOT(asrReleaseCallback()));

            //input_topic_editor->setText( input_topic );
            //setTopic();

            location_manager_.reset(new LocationManager);

            for(auto&it : location_manager_->getLocations())
            {
                location_box_->addItem(QString::fromStdString(it.first));
            }

            location_mark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tiffer_panel/Navigation/KnownLocations", 2, true);
            publishLocationsToRviz();
            
            cruise_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tiffer_panel/Navigation/CruisePath",2,true);
            nav_stop_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1, true);
            odom_sub_ = nh_.subscribe("/odom", 10, &TifferPanel::odomCallback, this);
            nav_status_sub_ = nh_.subscribe("/move_base/result", 10, &TifferPanel::statusCallback, this);
            path_len_sub_ = nh_.subscribe("/move_base/NavfnROS/plan", 10, &TifferPanel::globalPathLenCallback, this);
            nav_time_sub_ = nh_.subscribe("/move_base/current_goal", 10, &TifferPanel::remainNavTimeCallback, this);
            mouse_cruise_location_sub_ = nh_.subscribe("/tiffer_panel/MouseCruiseLocation", 10,
                                                        &TifferPanel::mouseCruiseLocationCallback, this);

            cruise_path_mark_.type = cruise_path_mark_.LINE_STRIP;
            cruise_path_mark_.action = cruise_path_mark_.ADD;
            cruise_path_mark_.header.frame_id = "map";
            cruise_path_mark_.points.clear();
            cruise_path_mark_.pose.orientation.w = 1;
            cruise_path_mark_.color.g = cruise_path_mark_.color.r = cruise_path_mark_.color.a = 1;
            cruise_path_mark_.scale.x = cruise_path_mark_.scale.y = cruise_path_mark_.scale.z = 0.05;
            cruise_number_mark_.markers.clear();
            cruise_path_.clear();

            stopCallback();     
        }

        template<class T>
        int arr_len(T& arr)
        {
            return sizeof(arr) / sizeof(arr[0]);
        }

        void TifferPanel::setTopic()
        {
            if(subscriber) {
                subscriber.shutdown();
            }
            input_topic = input_topic_editor->text();
            subscriber = nh.subscribe(std::string(input_topic.toStdString()), 100, &TifferPanel::message_cb, this);
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
            if(odom_.twist.twist.linear.x == 0) {
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
                std::cout << "it -> first: " << it.first << std::endl;
                QString tr1(tr(it.first.data()));

                if(typeid(it.first) == typeid(std::string)) 
                    std::cout << "it.first is std::string." << std::endl;

                std::cout << (typeid(tr1) == typeid(QString)) << std::endl;

                QString tr2 = tr1.toUtf8();
                qDebug() << "tr2 is :" << tr2;
                new_mark.text = tr2.toStdString();

                //new_mark.text = it.first;  //text -> string

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
            ROS_INFO_STREAM("Result" << msg->status);
            if(!in_cruise_mode_)
                setRobotStatus(NavStatus::IDLE);
            else if(msg->status.status == msg->status.SUCCEEDED)
            {
                std_msgs::Bool start_msg;
                start_msg.data = true;
                application_start_pub_.publish(start_msg);
                setRobotStatus(NavStatus::WAIT_APPLICATION);

                if(current_cruise_index_ > 0)
                {
                    location_widget_->item(current_cruise_index_ - 1)->setText(
                        QString::fromStdString(cruise_path_[current_cruise_index_ - 1].name) 
                            + QString::fromUtf8("   已抵达"));
                    location_widget_->item(current_cruise_index_ - 1)->setBackgroundColor(
                        QColor::fromRgb(200, 200, 200, 255));
                }
            }
            else {
                setRobotStatus(NavStatus::FAILED);
                ROS_ERROR_STREAM("Failed to cruise to location" << cruise_path_[current_cruise_index_ - 1].name);
            }
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

            std::string cur_path = ros::package::getPath("tiffer_panel");
            std::string rec_com = "arecord -c 1 -t wav -f S16_LE -r 16000 -d 4 " + cur_path + "/file/l";
            std::string asr_com = "python3 " + cur_path + "/python/asr.py";
            std::string file_com = cur_path + "/file/result";

            //if(0 == system("arecord -c 1 -t wav -f S16_LE -r 16000 -d 5 home/tiffer/tiffer-catkin/src/tiffer_panel/file/l")){
            if(0 == system(rec_com.data())){
                qDebug() << "record success.";
            }else{
                qDebug() << "record error.";
            }

            if(0 == system(asr_com.data())){
                qDebug() << "asr_bd_ol success.";
            }else{
                qDebug() << "asr_bd_ol failed.";
            }

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

            res_file.close();
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
                    status_line_->setText(QObject::trUtf8("没事干"));
                    status_line_->setStyleSheet(
                        "QLineEdit { background: rgb(0, 255, 255);"\
                        " color: rgb(0, 0, 0); font-size: 20px;}"
                    );
                    break;
                    
                case NavStatus::INPROGRESS:
                    status_line_->setText(QObject::trUtf8("走着呢"));
                    status_line_->setStyleSheet(
                        "QLineEdit { background: rgb(255, 0, 255);"\
                        " color: rgb(0, 0, 0); font-size: 20px;}"
                    );
                    break;

                case NavStatus::SUCCESS:
                    status_line_->setText(QObject::trUtf8("完成了"));
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
