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


        void TopicData::setLibrarySideFormat(esiaf_ros::AudioFormat librarySideFormat) {
            EsiafAudioFormat act;
            act.rate = esiaf_ros::Rate(librarySideFormat.rate);
            act.bitrate = esiaf_ros::Bitrate(librarySideFormat.bitrate);
            act.endian = esiaf_ros::Endian(librarySideFormat.endian);
            act.channels = librarySideFormat.channels;

            this->librarySideFormat = act;

            if (determine_resampling_necessary()) {
                ROS_DEBUG("creating resampler");
                initialize_resampler();
            } else if (this->resampler != nullptr) {
                ROS_DEBUG("getting rid of resampler");
                free(this->resampler);
            }
        }


        bool TopicData::determine_resampling_necessary() {
            ROS_DEBUG("rate, allowed: %d, actual: %d", (int) clientSideFormat.rate, (int) librarySideFormat.rate);
            ROS_DEBUG("bitrate, allowed: %d, actual: %d", (int) clientSideFormat.bitrate,
                      (int) librarySideFormat.bitrate);
            ROS_DEBUG("endian, allowed: %d, actual: %d", (int) clientSideFormat.endian, (int) librarySideFormat.endian);
            ROS_DEBUG("channel, allowed: %d, actual: %d", clientSideFormat.channels, librarySideFormat.channels);

            bool resampling_necessary = !(clientSideFormat.rate == librarySideFormat.rate
                                          && clientSideFormat.bitrate == librarySideFormat.bitrate
                                          && clientSideFormat.endian == librarySideFormat.endian
                                          && clientSideFormat.channels == librarySideFormat.channels);
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
                userCallback(callback_ptr),
                vadCallback(NULL) {
            this->topic = topic;
            this->clientSideFormat = topic.allowedFormat;
            this->librarySideFormat = topic.allowedFormat;
            this->subscriber = nodeHandle->subscribe<esiaf_ros::AugmentedAudio>(topic.topic, 1000, boost::bind(
                    &InputTopicData::internal_subscriber_callback, this, _1));
        }

        void InputTopicData::internal_subscriber_callback(const esiaf_ros::AugmentedAudio::ConstPtr &msg) {
            esiaf_ros::RecordingTimeStamps time = msg->time;
            std::vector<int8_t> signal = msg->signal;
            std::vector<int8_t> signal_correctly_sampled;
            // ignore channel for now
            if (determine_resampling_necessary()) {
                ROS_DEBUG("input topicdata resampling call");
                signal_correctly_sampled = this->resampler->resample(signal);
            } else {
                signal_correctly_sampled = signal;
            }

            try {
                ROS_DEBUG("invoke user callback");
                userCallback(signal_correctly_sampled, time);
            } catch (const std::exception &e) {
                ROS_INFO("%s", e.what());
            }
            try {
                if(msg->segmentmentation_ended && vadCallback != NULL)
                    vadCallback();
            } catch (const std::exception &e) {
                ROS_INFO("%s", e.what());
            }



        }

        void InputTopicData::addVADcallback(boost::function<void()> callback_ptr) {
            vadCallback = callback_ptr;
        }

        void InputTopicData::initialize_resampler() {
            this->resampler = new resampling::Resampler(this->librarySideFormat, this->clientSideFormat);
        }

        /////////////////////////////////////////////////////////////////////////////////////
        // OUTPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        OutputTopicData::OutputTopicData(ros::NodeHandle *nodeHandle, esiaf_ros::EsiafAudioTopicInfo topic) :
                vadFinished(false) {

            this->topic = topic;
            this->librarySideFormat = topic.allowedFormat;
            this->clientSideFormat = topic.allowedFormat;
            this->publisher = nodeHandle->advertise<esiaf_ros::AugmentedAudio>(topic.topic, 1000);
        }

        void OutputTopicData::publish(std::vector<int8_t> signal, esiaf_ros::RecordingTimeStamps timeStamps) {
            std::vector<int8_t> signal_correctly_sampled;
            if (determine_resampling_necessary()) {
                ROS_DEBUG("output topicdata resampling call");
                signal_correctly_sampled = this->resampler->resample(signal);
            } else {
                signal_correctly_sampled = signal;
            }
            esiaf_ros::AugmentedAudio msg;
            msg.signal = signal_correctly_sampled;
            msg.time = timeStamps;
            msg.channel = this->librarySideFormat.channels;
            msg.segmentmentation_ended = vadFinished;
            this->publisher.publish(msg);
            ROS_DEBUG("output topicdata publish complete");
            vadFinished = false;
        }

        void OutputTopicData::setVADfinished() {
            vadFinished = true;
        }

        void OutputTopicData::initialize_resampler() {
            this->resampler = new resampling::Resampler(this->clientSideFormat, this->librarySideFormat);
        }
    }
}