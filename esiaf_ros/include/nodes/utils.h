//
// Created by rfeldhans on 08.02.19.
//

#ifndef ESIAF_ROS_UTILS_H
#define ESIAF_ROS_UTILS_H

#include <string>
#include <regex>
#include <esiaf_ros.h>
#include <alsa/asoundlib.h>

namespace esiaf_ros {
    namespace utils {
        void autoExpandEnvironmentVariables(std::string &text);

        esiaf_ros::Bitrate cfg_to_esiaf_bitrate(int bitrate, char signed_value, char type_value);

        esiaf_ros::Endian cfg_to_esiaf_endian(std::string endian);

        esiaf_ros::Rate cfg_to_esaif_rate(int rate);

        /**
         * Takes an esiaf_ros Bitrate and Endian and turns them into the corresponding alsa (asoundlib) equivalent.
         * @param bitrate
         * @param endian
         * @return
         */
        _snd_pcm_format set_up_format_from_bitrate_and_endian(esiaf_ros::Bitrate bitrate, esiaf_ros::Endian endian);

        /**
         * Turns an esiaf_ros Rate into an unsigned int describing the same sample rate.
         * @param sample_rate
         * @return
         */
        unsigned int set_up_sample_rate_from_esiaf(esiaf_ros::Rate sample_rate);
    }
}


#endif //ESIAF_ROS_UTILS_H
