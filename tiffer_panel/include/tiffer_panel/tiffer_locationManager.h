///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 4/23/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit           
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TIFFER_LOCATIONMANAGER_H
#define TIFFER_LOCATIONMANAGER_H

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <map>

#endif

namespace tiffer_panel
{
    class KnownLocation
    {
        public:
            std::string name;
            geometry_msgs::Pose location;
    };

    class LocationManager
    {
        public:
            LocationManager();
            virtual ~LocationManager();

            bool initialise(const std::string &file);
            bool loadLocations(const std::string &file);
            bool saveLocations(const std::string &file);

            bool knownLocation(const std::string &name);
            bool getLocation(const std::string &name, geometry_msgs::Pose &location);
            std::map<std::string, KnownLocation> &getLocations();
            void addLocation(const std::string &name, const geometry_msgs::Pose &pose);
            void removeLocation(const std::string &name);
        protected:
            std::map<std::string, KnownLocation> locations_;
            std::map<std::string, KnownLocation> new_locations_;
    };

    typedef boost::shared_ptr<LocationManager> LocationManagerPtr;
}

#endif