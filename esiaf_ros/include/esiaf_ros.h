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
#include "esiaf_ros/AudioTopicFormatConstants.h"

namespace esiaf_ros {

    /**
     * An enum to determine what kind of node a specific node is.
     */
    enum class NodeDesignation{
        VAD = esiaf_ros::AudioTopicFormatConstants::VAD,
        SpeechRec = esiaf_ros::AudioTopicFormatConstants::SpeechRec,
        SSL = esiaf_ros::AudioTopicFormatConstants::SSL,
        Gender = esiaf_ros::AudioTopicFormatConstants::Gender,
        Emotion = esiaf_ros::AudioTopicFormatConstants::Emotion,
        VoiceId = esiaf_ros::AudioTopicFormatConstants::VoiceId,
        Other = esiaf_ros::AudioTopicFormatConstants::Other
    };

    /**
     * This enum is used to determine a audio rate a specific AudioFormat uses.
     */
    enum class Rate{
        RATE_8000 = esiaf_ros::AudioTopicFormatConstants::RATE_8000,
        RATE_16000 = esiaf_ros::AudioTopicFormatConstants::RATE_16000,
        RATE_32000 = esiaf_ros::AudioTopicFormatConstants::RATE_32000,
        RATE_44100 = esiaf_ros::AudioTopicFormatConstants::RATE_44100,
        RATE_48000 = esiaf_ros::AudioTopicFormatConstants::RATE_48000,
        RATE_96000 = esiaf_ros::AudioTopicFormatConstants::RATE_96000
    };

    /**
     * This enum is used to determine an AudioFormats bitrate.
     */
    enum class Bitrate{
        BIT_INT_8_SIGNED = esiaf_ros::AudioTopicFormatConstants::BIT_INT_8_SIGNED,
        BIT_INT_8_UNSIGNED = esiaf_ros::AudioTopicFormatConstants::BIT_INT_8_UNSIGNED,
        BIT_INT_16_SIGNED = esiaf_ros::AudioTopicFormatConstants::BIT_INT_16_SIGNED,
        BIT_INT_16_UNSIGNED = esiaf_ros::AudioTopicFormatConstants::BIT_INT_16_UNSIGNED,
        BIT_INT_24_SIGNED = esiaf_ros::AudioTopicFormatConstants::BIT_INT_24_SIGNED,
        BIT_INT_24_UNSIGNED = esiaf_ros::AudioTopicFormatConstants::BIT_INT_24_UNSIGNED,
        BIT_INT_32_SIGNED = esiaf_ros::AudioTopicFormatConstants::BIT_INT_32_SIGNED,
        BIT_INT_32_UNSIGNED = esiaf_ros::AudioTopicFormatConstants::BIT_INT_32_UNSIGNED,
        BIT_FLOAT_32 = esiaf_ros::AudioTopicFormatConstants::BIT_FLOAT_32,
        BIT_FLOAT_64 = esiaf_ros::AudioTopicFormatConstants::BIT_FLOAT_64
    };

    /**
     * This enum is used to determine an AudioFormats endian.
     */
    enum class Endian{
        LittleEndian = esiaf_ros::AudioTopicFormatConstants::LittleEndian,
        BigEndian = esiaf_ros::AudioTopicFormatConstants::BigEndian
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
     * A struct for the esiaf handle. Used primarily to make all methods stateless and reentrant.
     */
    struct esiaf_handle;

    class Esiaf_Handler {
    public:

        Esiaf_Handler(ros::NodeHandle *nodeHandle,
                      NodeDesignation nodeDesignation);

        void add_input_topic(EsiafAudioTopicInfo &input,
                             boost::function<void(
                                     const std::vector<int8_t>&,
                                     const esiaf_ros::RecordingTimeStamps&)>
                             callback);

        void add_output_topic(EsiafAudioTopicInfo &output);

        void start_esiaf();

        void publish(std::string topic,
                     std::vector<int8_t> signal,
                     esiaf_ros::RecordingTimeStamps timeStamps);

        void add_vad_finished_callback(std::string topic,
                                       boost::function<void()> callback);

        void set_vad_finished(std::string topic);

        void quit_esiaf();


    private:
        esiaf_handle* handle;

    };

}// namespace

#endif //ESIAF_LIBRARY_H
