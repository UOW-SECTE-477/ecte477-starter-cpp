/**
    commands.cpp

    Defines an enum and map for valid commands to be sent over cmd/ topic

    Using an enum whose values are equal to the strings we expect via a map
     is a safer method than using the strings themselves. With string alone,
    typos won't result in a error but our program won't work. Using Enums,
    a typo will result in an error which will be easier to fix.

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
    namespace Commands
    {
        static std::string const START = "start";
        static std::string const STOP = "stop";
    } // namespace Commands
    namespace RobotState
    {
        static std::string const WAITING_TO_START = "waiting_to_start";
        static std::string const PAUSED = "paused";
        static std::string const EXPLORE = "explore";
        static std::string const RETURNING = "returning";
        static std::string const DONE = "done";
    } // namespace RobotState
} // namespace ecte477