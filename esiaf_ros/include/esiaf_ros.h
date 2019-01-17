//
// Created by rfeldhans on 03.11.18.
//

#ifndef ESIAF_LIBRARY_H
#define ESIAF_LIBRARY_H

// std includes
#include <string>
#include <vector>
#include <functional>

// ros includes
#include "ros/ros.h"
#include "esiaf_ros/RecordingTimeStamps.h"

namespace esiaf_ros {
    enum class NodeDesignation{
        VAD = 1,
        SpeechRec = 2,
        SSL = 3,
        Gender = 4,
        Emotion = 5,
        VoiceId = 6,
        Other = 7
    };

    enum class Rate{
        RATE_8000 = 8,
        RATE_16000 = 16,
        RATE_32000 = 32,
        RATE_44100 = 44,
        RATE_48000 = 48,
        RATE_96000 = 96
    };

    enum class Bitrate{
        BIT_8 = 8,
        BIT_16 = 16,
        BIT_24 = 24,
        BIT_32 = 32
    };

    enum class Endian{
        LittleEndian = 1,
        BigEndian = 10
    };

    struct EsiafAudioFormat {
        Rate rate;
        Bitrate bitrate;
        int channels;
        Endian endian;
    };

    struct EsiafAudioTopicInfo {
        std::string topic;
        EsiafAudioFormat allowedFormat;
    };

    void initialize_esiaf(ros::NodeHandle *nodeHandle, NodeDesignation nodeDesignation);

    void add_input_topic(EsiafAudioTopicInfo &input, std::function<void(char*, size_t, esiaf_ros::RecordingTimeStamps)>);

    void add_output_topic(EsiafAudioTopicInfo &output);

    void start_esiaf();

    void publish(std::string topic, char *signalBuffer, size_t buffersize, esiaf_ros::RecordingTimeStamps timeStamps);

}// namespace

#endif //ESIAF_LIBRARY_H
