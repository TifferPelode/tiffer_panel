///////////////////////////////////////////////////////////////////////////////
//      Title     : Status logger panel
//      Project   : ROSSTEP
//      Created   : 7/15/2015
//      Author    : Adam Allevato
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
////////////////////////////////////////////////////////////////////////////////

#include "tiffer_panel/tiffer_panel.h"

#include <stdio.h>

#include <geometry_msgs/Twist.h>

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
  message_display = new QLabel("tiffer pelode");
  message_display->setTextFormat(Qt::RichText);
  message_display->setAlignment(Qt::AlignCenter);

  // Lay out the topic field next to the control widrivzget.
  //QGridLayout* layout = new QGridLayout();
  //layout->setColumnStretch(1,100);
  //layout->addWidget( message_display, 0,0,1,4);
  //layout->addLayout( topic_layout, 0,4,1,1 );
  main_layout->addLayout(topic_layout);
  main_layout->addWidget(message_display);
  addLine(main_layout);
  setLayout( main_layout );

  input_topic_editor->resize(150, input_topic_editor->height());

  // Next we make signal/slot connections.
  connect( input_topic_editor, SIGNAL( editingFinished() ), this, SLOT( setTopic() ));

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
