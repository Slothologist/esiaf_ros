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

    /**
     * An enum to determine what kind of node a specific node is.
     */
    enum class NodeDesignation{
        VAD = 1,
        SpeechRec = 2,
        SSL = 3,
        Gender = 4,
        Emotion = 5,
        VoiceId = 6,
        Other = 7
    };

    /**
     * This enum is used to determine a audio rate a specific AudioFormat uses.
     */
    enum class Rate{
        RATE_8000 = 8,
        RATE_16000 = 16,
        RATE_32000 = 32,
        RATE_44100 = 44,
        RATE_48000 = 48,
        RATE_96000 = 96
    };

    /**
     * This enum is used to determine an AudioFormats bitrate.
     */
    enum class Bitrate{
        BIT_8 = 8,
        BIT_16 = 16,
        BIT_24 = 24,
        BIT_32 = 32
    };

    /**
     * This enum is used to determine an AudioFormats endian.
     */
    enum class Endian{
        LittleEndian = 1,
        BigEndian = 10
    };

    /**
     * This struct is used to store an audio format. It is comprised of a rate and bitrate, the amount of channels and
     * the endian of the audio. Each of these values (apart from the amount of channels) is determined by another enum.
     */
    struct EsiafAudioFormat {
        Rate rate;
        Bitrate bitrate;
        int channels;
        Endian endian;
    };

    /**
     * Struct to store information about a esiaf_ros audio topic. Consists of a topic name and a format.
     */
    struct EsiafAudioTopicInfo {
        std::string topic;
        EsiafAudioFormat allowedFormat;
    };

    /**
     *
     * @param nodeHandle
     * @param nodeDesignation
     */
    void initialize_esiaf(ros::NodeHandle *nodeHandle, NodeDesignation nodeDesignation);

    /**
     *
     * @param input
     * @param callback
     */
    void add_input_topic(EsiafAudioTopicInfo &input,
                         const std::function<void(std::vector<int8_t>, esiaf_ros::RecordingTimeStamps)>& callback);

    /**
     *
     * @param output
     */
    void add_output_topic(EsiafAudioTopicInfo &output);

    /**
     *
     */
    void start_esiaf();

    /**
     *
     * @param topic
     * @param signalBuffer
     * @param buffersize
     * @param timeStamps
     */
    void publish(std::string topic, std::vector<int8_t> signal, esiaf_ros::RecordingTimeStamps timeStamps);

}// namespace

#endif //ESIAF_LIBRARY_H
