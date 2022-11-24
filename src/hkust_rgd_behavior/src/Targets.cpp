#include <hkust_rgd_behavior/Targets.hpp>

void mapStr2Target(const char *str, Target *targetOut)
{
    if (std::string(str).find("chair") != std::string::npos)
    {
        *targetOut = Target::TARGET_CHAIR;
    }
    else if (std::string(str).find("stair") != std::string::npos)
    {
        *targetOut = Target::TARGET_STAIR;
    }
    else if (std::string(str).find("door") != std::string::npos)
    {
        *targetOut = Target::TARGET_DOOR;
    }
    else
    {
        *targetOut = Target::TARGET_NONE;
    }
}