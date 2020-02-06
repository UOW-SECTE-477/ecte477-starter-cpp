/**
    commands.cpp

    Defines an enum and map for valid colours for the beacons

    Created: 2020/02/06
    Author: Brendan Halloran
**/

// Standard Library
#include <string>
#include <unordered_map>

// External Libraries

// Local Headers

namespace ecte477
{
    enum class Colours
    {
        RED,
        GREEN,
        BLUE,
        YELLOW
    };

    std::unordered_map<std::string, Colours> colours = {{"red", Colours::RED},
                                                        {"green", Colours::GREEN},
                                                        {"blue", Colours::BLUE},
                                                        {"yellow", Colours::YELLOW}};
} // namespace ecte477