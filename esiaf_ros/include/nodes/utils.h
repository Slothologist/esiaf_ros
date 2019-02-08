//
// Created by rfeldhans on 08.02.19.
//

#ifndef ESIAF_ROS_UTILS_H
#define ESIAF_ROS_UTILS_H

#include <string>
#include <regex>
#include <esiaf_ros.h>

namespace esiaf_ros {
    namespace utils {
        void autoExpandEnvironmentVariables(std::string &text);

        esiaf_ros::Bitrate cfg_to_esiaf_bitrate(int bitrate);

        esiaf_ros::Endian cfg_to_esiaf_endian(std::string endian);

        esiaf_ros::Rate cfg_to_esaif_rate(int rate);
    }
}


#endif //ESIAF_ROS_UTILS_H
