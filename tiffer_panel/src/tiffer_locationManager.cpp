///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 4/23/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit           
//
////////////////////////////////////////////////////////////////////////////////

#include "tiffer_panel/tiffer_locationManager.h"

namespace tiffer_panel
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

    bool LocationManager::knownLocation(const std::string &name)
    {
        if(locations_.find(name) != locations_.end())
            return true;
        return new_locations_.find(name) != new_locations_.end();
    }
}