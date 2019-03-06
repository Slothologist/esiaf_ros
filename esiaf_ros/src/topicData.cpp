//
// Created by rfeldhans on 09.01.19.
//
#include "../include/topicData.h"

#include "esiaf_ros/RecordingTimeStamps.h"

namespace esiaf_ros {
    namespace topicdata {

        /////////////////////////////////////////////////////////////////////////////////////
        // BASE CLASS
        /////////////////////////////////////////////////////////////////////////////////////

        std::string TopicData::getTopicName() {
            return this->topic.topic;
        }

        esiaf_ros::EsiafAudioTopicInfo TopicData::getInfo() {
            return this->topic;
        }


        void TopicData::setActualFormat(esiaf_ros::AudioFormat actualFormat) {
            EsiafAudioFormat act;
            act.rate = esiaf_ros::Rate(actualFormat.rate);
            act.bitrate = esiaf_ros::Bitrate(actualFormat.bitrate);
            act.endian = esiaf_ros::Endian(actualFormat.endian);
            act.channels = actualFormat.channels;

            this->actualFormat = act;

            if (determine_resampling_necessary()) {
                ROS_DEBUG("should create resampler");
                initialize_resampler();
            } else if (this->resampler != nullptr) {
                ROS_INFO("getting rid of resampler");
                free(this->resampler);
            }
        }


        bool TopicData::determine_resampling_necessary() {
            ROS_DEBUG("rate, allowed: %d, actual: %d", (int)topic.allowedFormat.rate, (int)actualFormat.rate);
            ROS_DEBUG("bitrate, allowed: %d, actual: %d", (int)topic.allowedFormat.bitrate, (int)actualFormat.bitrate);
            ROS_DEBUG("endian, allowed: %d, actual: %d", (int)topic.allowedFormat.endian, (int)actualFormat.endian);
            ROS_DEBUG("channel, allowed: %d, actual: %d", topic.allowedFormat.channels, actualFormat.channels);

            bool resampling_necessary = !(topic.allowedFormat.rate == actualFormat.rate
                                     && topic.allowedFormat.bitrate == actualFormat.bitrate
                                     && topic.allowedFormat.endian == actualFormat.endian
                                     && topic.allowedFormat.channels == actualFormat.channels);
            ROS_DEBUG("set resampling necessary? %d", resampling_necessary);
            return resampling_necessary;
        }

        /////////////////////////////////////////////////////////////////////////////////////
        // INPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        InputTopicData::InputTopicData(ros::NodeHandle *nodeHandle,
                                       esiaf_ros::EsiafAudioTopicInfo topic,
                                       boost::function<void(const std::vector<int8_t> &,
                                                            const esiaf_ros::RecordingTimeStamps &)> callback_ptr) :
                userCallback(callback_ptr) {
            this->topic = topic;
            this->actualFormat = topic.allowedFormat;
            this->subscriber = nodeHandle->subscribe<esiaf_ros::AugmentedAudio>(topic.topic, 1000, boost::bind(
                    &InputTopicData::internal_subscriber_callback, this, _1));
        }

        void InputTopicData::internal_subscriber_callback(const esiaf_ros::AugmentedAudio::ConstPtr &msg) {
            esiaf_ros::RecordingTimeStamps time = msg->time;
            std::vector<int8_t> signal = msg->signal;
            // ignore channel for now
            if (determine_resampling_necessary()) {
                ROS_INFO("input topicdata resampling call");
                signal = this->resampler->resample(signal);
            }
            try {
                userCallback(signal, time);
            } catch (const std::exception &e) {
                ROS_INFO("%s", e.what());
            }

        }

        void InputTopicData::initialize_resampler() {
            this->resampler = new resampling::Resampler(this->actualFormat, this->topic.allowedFormat);
        }

        /////////////////////////////////////////////////////////////////////////////////////
        // OUTPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        OutputTopicData::OutputTopicData(ros::NodeHandle *nodeHandle, esiaf_ros::EsiafAudioTopicInfo topic) {

            this->topic = topic;
            this->actualFormat = topic.allowedFormat;
            this->publisher = nodeHandle->advertise<esiaf_ros::AugmentedAudio>(topic.topic, 1000);
        }

        void OutputTopicData::publish(std::vector<int8_t> signal, esiaf_ros::RecordingTimeStamps timeStamps) {

            if (determine_resampling_necessary()) {
                ROS_INFO("output topicdata resampling call");
                signal = this->resampler->resample(signal);
            }
            esiaf_ros::AugmentedAudio msg;
            msg.signal = signal;
            msg.time = timeStamps;
            msg.channel = this->actualFormat.channels;
            this->publisher.publish(msg);
        }

        void OutputTopicData::initialize_resampler() {
            this->resampler = new resampling::Resampler(this->topic.allowedFormat, this->actualFormat);
        }
    }
}