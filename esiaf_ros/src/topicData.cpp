//
// Created by rfeldhans on 09.01.19.
//
#include "../include/topicData.h"

#include "esiaf_ros/RecordingTimeStamps.h"

namespace esiaf_ros{
    namespace topicdata{

        /////////////////////////////////////////////////////////////////////////////////////
        // BASE CLASS
        /////////////////////////////////////////////////////////////////////////////////////

        std::string TopicData::getTopicName(){
            return topic.topic;
        }

        esiaf_ros::EsiafAudioTopicInfo TopicData::getInfo(){
            return topic;
        }


        void TopicData::setActualFormat(esiaf_ros::AudioFormat actualFormat){
            this->actualFormat = actualFormat;
            determine_resampling_necessary();
        }


        void TopicData::determine_resampling_necessary() {
            resampling_necessary = topic.allowedFormat.rate == esiaf_ros::Rate(actualFormat.rate)
                    && topic.allowedFormat.bitrate == esiaf_ros::Bitrate(actualFormat.bitrate)
                    && topic.allowedFormat.endian == esiaf_ros::Endian(actualFormat.endian)
                    && topic.allowedFormat.channels == actualFormat.channels;
        }

        /////////////////////////////////////////////////////////////////////////////////////
        // INPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        InputTopicData::InputTopicData(ros::NodeHandle* nodeHandle,
                                       esiaf_ros::EsiafAudioTopicInfo topic,
                                       boost::function<void(const std::vector<int8_t>&,
                                                                const esiaf_ros::RecordingTimeStamps&)> callback_ptr):
                userCallback(callback_ptr){
            this->topic = topic;
            subscriber = nodeHandle->subscribe<esiaf_ros::AugmentedAudio>(topic.topic, 1000, boost::bind(&InputTopicData::internal_subscriber_callback, this, _1));
        }

        void InputTopicData::internal_subscriber_callback(const esiaf_ros::AugmentedAudio::ConstPtr& msg){
            esiaf_ros::RecordingTimeStamps time = msg->time;
            std::vector<int8_t> signal = msg->signal;
            // ignore channel for now
            if(resampling_necessary){
                //todo
            }
            try {
                userCallback(signal, time);
            }catch (const std::exception& e){
                ROS_INFO("%s", e.what());
            }

        }

        /////////////////////////////////////////////////////////////////////////////////////
        // OUTPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        OutputTopicData::OutputTopicData(ros::NodeHandle* nodeHandle, esiaf_ros::EsiafAudioTopicInfo topic) {

            this->topic = topic;
            publisher = nodeHandle->advertise<esiaf_ros::AugmentedAudio>(topic.topic, 1000);
        }

        void OutputTopicData::publish(std::vector<int8_t> signal, esiaf_ros::RecordingTimeStamps timeStamps) {

            if(resampling_necessary){
                //todo
            }
            esiaf_ros::AugmentedAudio msg;
            msg.signal = signal;
            msg.time = timeStamps;
            msg.channel = actualFormat.channels;
            //std::vector<char> vec = std::vector(signalBuffer);
            // ignore channel for now
            publisher.publish(msg);
        }
    }
}