#pragma once
#include <iostream>
enum Action
{
    ACTION_MOVE_RIGHT,
    ACTION_MOVE_LEFT,
    ACTION_MOVE_FORWARD,
    ACTION_FIND,
    ACTION_STOP,
    ACTION_NONE,
    ACTION_VOICE
};

void mapStr2Action(const char *str, Action *actionOut);