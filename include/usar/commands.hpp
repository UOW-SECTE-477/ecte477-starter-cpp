/**
    commands.cpp

    Defines an enum and map for valid commands to be sent over cmd/ topic

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
    enum class Commands
    {
        START,
        EXPLORE,
        RETURN,
        HALT
    };

    std::unordered_map<std::string, Commands> commands = {{"start", Commands::START},
                                                          {"explore", Commands::EXPLORE},
                                                          {"return", Commands::RETURN},
                                                          {"halt", Commands::HALT}};
} // namespace ecte477