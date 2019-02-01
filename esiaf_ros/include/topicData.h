//
// Created by rfeldhans on 09.01.19.
//

#ifndef ESIAF_ROS_TOPICDATA_H
#define ESIAF_ROS_TOPICDATA_H

#include "ros/ros.h"
#include "../include/esiaf_ros.h"
#include "esiaf_ros/AugmentedAudio.h"
#include "esiaf_ros/AudioFormat.h"
#include "esiaf_ros/RecordingTimeStamps.h"

namespace esiaf_ros{
    namespace topicdata{

        /////////////////////////////////////////////////////////////////////////////////////
        // BASE CLASS
        /////////////////////////////////////////////////////////////////////////////////////

        class TopicData {

        public:
            std::string getTopicName();
            esiaf_ros::EsiafAudioTopicInfo getInfo();
            void setActualFormat(esiaf_ros::AudioFormat actualFormat);

        protected:
            esiaf_ros::EsiafAudioTopicInfo topic;

            bool resampling_necessary = false;
            esiaf_ros::AudioFormat actualFormat;

        private:
            void determine_resampling_necessary();

        };

        /////////////////////////////////////////////////////////////////////////////////////
        // INPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        class InputTopicData: public TopicData{

        public:
            InputTopicData(ros::NodeHandle* nodeHandle,
                           esiaf_ros::EsiafAudioTopicInfo topic,
                           boost::function<void(const std::vector<int8_t>&,
                                              const esiaf_ros::RecordingTimeStamps&)> callback_ptr);

        private:
            ros::Subscriber subscriber;

            boost::function<void(const std::vector<int8_t>&, const esiaf_ros::RecordingTimeStamps&)> userCallback;

            void internal_subscriber_callback(const esiaf_ros::AugmentedAudio::ConstPtr& msg);
        };


        /////////////////////////////////////////////////////////////////////////////////////
        // OUTPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        class OutputTopicData: public TopicData{

        public:
            OutputTopicData(ros::NodeHandle* nodeHandle, esiaf_ros::EsiafAudioTopicInfo topic);

            void publish(std::vector<int8_t> signal, esiaf_ros::RecordingTimeStamps timeStamps);

        private:
            ros::Publisher publisher;
        };

    }
}

#endif //ESIAF_ROS_TOPICDATA_H
