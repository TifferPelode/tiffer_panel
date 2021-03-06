///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 4/23/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit           
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TIFFER_CRUISELOCATION_H
#define TIFFER_CRUISELOCATION_H

#include <ros/ros.h>
#include "rviz/default_plugin/tools/pose_tool.h"

#include <QObject>
#include <QInputDialog>

using namespace rviz;

namespace Tiffer
{
    class Arrow;
    class DisplayContext;
    class StringProperty;

    class CruiseLocationTool : public PoseTool
    {
        Q_OBJECT
        public:
            CruiseLocationTool();
            virtual void onInitialize();
        
        protected:
            virtual void onPoseSet(double x, double y, double theta);
        
        private Q_SLOTS:
            void updateTopic();
        
        private:
            ros::NodeHandle nh_;
            ros::Publisher pub_;
    };
}

#endif /* TIFFER_CRUISELOCATION_H */