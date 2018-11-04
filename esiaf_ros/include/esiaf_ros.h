//
// Created by rfeldhans on 03.11.18.
//

#ifndef ESIAF_LIBRARY_H
#define ESIAF_LIBRARY_H

#include "ros/ros.h"

namespace esiaf_ros{

    struct EsiafAudioFormat{
        int rate;
        int bitrate;
        int channels;
        std::string endian;
    };

    struct EsiafAudioTopicInfo{
        std::string topic;
        std::vector<EsiafAudioFormat> allowedFormats;
    };

    void initialize_esiaf(ros::NodeHandle* nodeHandle);

    void add_input_topic(EsiafAudioTopicInfo &input);

    void add_output_topic(EsiafAudioTopicInfo &output);

    void start_esiaf();
}// namespace

#endif //ESIAF_LIBRARY_H
