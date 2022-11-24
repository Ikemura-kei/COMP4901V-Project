#pragma once
#include <iostream>
enum Target
{
    TARGET_CHAIR,
    TARGET_STAIR,
    TARGET_DOOR,
    TARGET_NONE
};

void mapStr2Target(const char *str, Target *targetOut);