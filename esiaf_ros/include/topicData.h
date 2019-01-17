//
// Created by rfeldhans on 09.01.19.
//

#ifndef ESIAF_ROS_TOPICDATA_H
#define ESIAF_ROS_TOPICDATA_H

#include "ros/ros.h"
#include "../include/esiaf_ros.h"
#include "esiaf_ros/AugmentedAudio.h"
#include "esiaf_ros/RecordingTimeStamps.h"

namespace esiaf_ros{
    namespace topicdata{


        class TopicData {

        public:
            std::string getTopicName();

        protected:
            esiaf_ros::EsiafAudioTopicInfo topic;

        };

        class InputTopicData: TopicData{

        public:
            std::string getTopicName();

            InputTopicData(ros::NodeHandle* nodeHandle, esiaf_ros::EsiafAudioTopicInfo topic, std::function<void(char*, size_t, esiaf_ros::RecordingTimeStamps)> callback_ptr);

        private:
            ros::Subscriber subscriber;

            void internal_subscriber_callback(const esiaf_ros::AugmentedAudio::ConstPtr& msg);
        };


        class OutputTopicData: TopicData{

        public:
            OutputTopicData(ros::NodeHandle* nodeHandle, esiaf_ros::EsiafAudioTopicInfo topic);

            std::string getTopicName();

            void publish(char *signalBuffer, size_t buffersize, esiaf_ros::RecordingTimeStamps);

        private:
            ros::Publisher publisher;
        };

    }
}

#endif //ESIAF_ROS_TOPICDATA_H
