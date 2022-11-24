#include <hkust_rgd_behavior/Locations.hpp>

void mapStr2Location(const char *str, Location *locationOut)
{
    if (std::string(str).find("supermarket") != std::string::npos)
    {
        *locationOut = Location::LOCATION_SUPERMARKET;
    }
    else
    {
        *locationOut = Location::LOCATION_NONE;
    }
}