#pragma once
#include <iostream>
enum Location
{
    LOCATION_SUPERMARKET,
    LOCATION_NONE
};

void mapStr2Location(const char *str, Location *locationOut);