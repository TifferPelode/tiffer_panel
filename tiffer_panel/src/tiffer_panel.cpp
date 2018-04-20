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
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Status Topic:" ));
  input_topic_editor = new QLineEdit;
  topic_layout->addWidget( input_topic_editor );

  //QHBoxLayout* message_layout = new QHBoxLayout;
  message_display = new QLabel(QObject::trUtf8("智澜科技"));
  message_display->setTextFormat(Qt::RichText);
  message_display->setAlignment(Qt::AlignCenter); 

  //Localize button
  QPushButton* localize_button = new QPushButton(QObject::trUtf8("自动定位"));
  main_layout->addWidget(localize_button);
  
  addLine(main_layout);
  main_layout->addWidget(message_display);

  setLayout( main_layout );

  input_topic_editor->resize(150, input_topic_editor->height());

  // Next we make signal/slot connections.
  //connect( input_topic_editor, SIGNAL( editingFinished() ), this, SLOT( setTopic() ));
  connect(localize_button, SIGNAL(clicked()), this, SLOT(localizeCallback()));

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
  QMessageBox confirmation(QMessageBox::Question, QObject::trUtf8("提示"), QObject::trUtf8("一方风景"));
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
