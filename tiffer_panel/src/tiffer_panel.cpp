///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 4/20/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit           
//
////////////////////////////////////////////////////////////////////////////////

#include "tiffer_panel/tiffer_panel.h"

namespace tiffer_panel {

TifferPanel::TifferPanel( QWidget* parent ) :
  rviz::Panel( parent ),
  input_topic("/status")
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

	QPushButton* nav_button = new QPushButton(QObject::trUtf8("导航至选中位置"));
	main_layout->addWidget(nav_button);
	QPushButton* stop_button = new QPushButton(QObject::trUtf8("停止"));
	main_layout->addWidget(stop_button);
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

    input_topic_editor->resize(150, input_topic_editor->height());

    // Next we make signal/slot connections.
    //connect( input_topic_editor, SIGNAL( editingFinished() ), this, SLOT( setTopic() ));
    connect(localize_button, SIGNAL(clicked()), this, SLOT(localizeCallback()));
    connect(add_location_button, SIGNAL(clicked()), this, SLOT(addLocationCallback()));
    connect(remove_location_button, SIGNAL(clicked()), this, SLOT(removeLocationCallback()));
	connect(nav_button, SIGNAL(clicked()), this, SLOT(navigationCallback()));
    connect(stop_button, SIGNAL(clicked()), this, SLOT(stopCallback()));

    input_topic_editor->setText( input_topic );
    setTopic();
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
        accept.setText(trUtf8("已确认desu"));
        accept.exec();
    }
    else if(confirmation.clickedButton() == no){
        QMessageBox reject;
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

bool TifferPanel::addLocation(const geometry_msgs::Pose &pose, const std::string &name)
{
    //if(location_manager_)
}

void TifferPanel::removeLocationCallback()
{

}

void TifferPanel::navigationCallback()
{
	std::string location_name_ = location_box_->currentText().toStdString();
}

void TifferPanel::stopCallback()
{
    actionlib_msgs::GoalID stop_msg;
    //nav_stop_pub_.publish(stop_msg);
    qDebug() << "stop";
}

void TifferPanel::addLine(QVBoxLayout* layout)
{
    QFrame *line = new QFrame;
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    line->setLineWidth(1);
    layout->addWidget(line);
}


} // end namespace

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tiffer_panel::TifferPanel,rviz::Panel )
