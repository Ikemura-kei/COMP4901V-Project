#include <hkust_rgd_behavior/Actions.hpp>

void mapStr2Action(const char *str, Action *actionOut)
{
    if (std::string(str).find("right") != std::string::npos)
    {
        *actionOut = Action::ACTION_MOVE_RIGHT;
    }
    else if (std::string(str).find("left") != std::string::npos)
    {
        *actionOut = Action::ACTION_MOVE_LEFT;
    }
    else if (std::string(str).find("forward") != std::string::npos)
    {
        *actionOut = Action::ACTION_MOVE_FORWARD;
    }
    else if (std::string(str).find("find") != std::string::npos)
    {
        *actionOut = Action::ACTION_FIND;
    }
    else if (std::string(str).find("stop") != std::string::npos)
    {
        *actionOut = Action::ACTION_STOP;
    }
    else if (std::string(str).find("voice") != std::string::npos)
    {
        *actionOut = Action::ACTION_VOICE;
    }
    else
    {
        *actionOut = Action::ACTION_NONE;
    }
}

