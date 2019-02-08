//
// Created by rfeldhans on 08.02.19.
//

#ifndef ESIAF_ROS_UTILS_H
#define ESIAF_ROS_UTILS_H

#include <string>
#include <regex>

namespace esiaf_ros {
    namespace utils {
        void autoExpandEnvironmentVariables(std::string &text);
    }
}


#endif //ESIAF_ROS_UTILS_H
