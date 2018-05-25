///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 4/23/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit           
//
////////////////////////////////////////////////////////////////////////////////

#include "tiffer_panel/tiffer_locationManager.h"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

namespace Tiffer
{
    namespace Navigation
    {
        LocationManager::LocationManager()
        {

        }

        LocationManager::~LocationManager()
        {
            
        }

        bool LocationManager::initialise(const std::string &file)
        {
            return true;
        }

        bool LocationManager::knowLocation(const std::string &name)
        {
            if(locations_.find(name) != locations_.end())
                return true;
            return new_locations_.find(name) != new_locations_.end();
        }

        bool LocationManager::getLocation(const std::string &name, geometry_msgs::Pose &location)
        {
            for(auto &it : locations_){
                if(it.first == name){
                    location = it.second.location;
                    return true;
                }
            }

            for(auto &it : new_locations_){
                if(it.first == name){
                    location = it.second.location;
                    return true;
                }
            }

            return false;
        }

        std::map<std::string, KnownLocation> &LocationManager::getLocations()
        {
            return locations_;
        }

        void LocationManager::addLocation(const std::string &name, const geometry_msgs::Pose &pose)
        {
            locations_[name].location = pose;
        }

        void LocationManager::removeLocation(const std::string &name)
        {
            locations_.erase(name);
        }

        bool LocationManager::loadLocations(const std::string &file)
        {
            ROS_INFO("Loading known locations");
            locations_.clear();
            std::string line;
            std::ifstream location_file(file);
            if(!location_file.is_open())
            {
                ROS_ERROR_STREAM("Failed to load file from " << file);
                return false;
            }

            while(getline(location_file, line))
            {
                std::vector<std::string> strs;
                boost::split(strs, line, boost::is_any_of(" "));
                if(strs.size() != 8)
                {
                    ROS_ERROR_STREAM("Incorrent line size " << strs.size() << " != 8");
                    return false;
                }
                KnownLocation new_location;
                new_location.name = strs[0];
                new_location.location.position.x = std::stod(strs[1]);
                new_location.location.position.y = std::stod(strs[2]);
                new_location.location.position.z = std::stod(strs[3]);
                new_location.location.orientation.x = std::stod(strs[4]);
                new_location.location.orientation.y = std::stod(strs[5]);
                new_location.location.orientation.z = std::stod(strs[6]);
                new_location.location.orientation.w = std::stod(strs[7]);

                locations_[new_location.name] = new_location;
                //HIGHLIGHT_NAMED("Location Manager", "Find new location" << new_location.name);
            }
            location_file.close();
            return true;
        }

        bool LocationManager::saveLocations(const std::string &file)
        {
            std::ofstream location_file(file);
            if(!location_file.is_open())
            {
                ROS_ERROR_STREAM("Fail to open location file " << file);
                return false;
            }
            
            for(auto &it : locations_)
            {
                location_file << it.first << " " << it.second.location.position.x << " "
                    << it.second.location.position.y << " " << it.second.location.position.z << " "
                    << it.second.location.orientation.x << " " << it.second.location.orientation.y << " "
                    << it.second.location.orientation.z << " " << it.second.location.orientation.w << std::endl;
            }
            location_file.close();
            return true;
        }
    }
}