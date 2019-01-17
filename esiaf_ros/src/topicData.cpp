//
// Created by rfeldhans on 09.01.19.
//
#include "../include/topicData.h"
#include <assert.h>

namespace esiaf_ros{
    namespace topicdata{

        /////////////////////////////////////////////////////////////////////////////////////
        // BASE CLASS
        /////////////////////////////////////////////////////////////////////////////////////

        std::string TopicData::getTopicName(){
            return topic.topic;
        }

        /////////////////////////////////////////////////////////////////////////////////////
        // OUTPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        OutputTopicData::OutputTopicData(ros::NodeHandle* nodeHandle, esiaf_ros::EsiafAudioTopicInfo topic) {
            this->topic = topic;
            publisher = nodeHandle->advertise<esiaf_ros::AugmentedAudio>(topic.topic, 1000);
        }

        void OutputTopicData::publish(char *signalBuffer, size_t buffersize, esiaf_ros::RecordingTimeStamps) {

        }

        /////////////////////////////////////////////////////////////////////////////////////
        //INPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        InputTopicData::InputTopicData(ros::NodeHandle* nodeHandle, esiaf_ros::EsiafAudioTopicInfo topic, std::function<void(char*, size_t, esiaf_ros::RecordingTimeStamps)> callback_ptr) {
            this->topic = topic;
            subscriber = nodeHandle->subscribe<esiaf_ros::AugmentedAudio>(topic.topic, 1000, boost::bind(&InputTopicData::internal_subscriber_callback, this, _1));
        }

        void InputTopicData::internal_subscriber_callback(const esiaf_ros::AugmentedAudio::ConstPtr& msg){

        }
    }
}