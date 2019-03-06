//
// Created by rfeldhans on 09.01.19.
//

#ifndef ESIAF_ROS_TOPICDATA_H
#define ESIAF_ROS_TOPICDATA_H

#include "ros/ros.h"
#include "esiaf_ros.h"
#include "esiaf_ros/AugmentedAudio.h"
#include "esiaf_ros/AudioFormat.h"
#include "esiaf_ros/RecordingTimeStamps.h"
#include "resample.h"

namespace esiaf_ros {
    namespace topicdata {

        /////////////////////////////////////////////////////////////////////////////////////
        // BASE CLASS
        /////////////////////////////////////////////////////////////////////////////////////

        class TopicData {

        public:
            std::string getTopicName();

            esiaf_ros::EsiafAudioTopicInfo getInfo();

            void setActualFormat(esiaf_ros::AudioFormat actualFormat);
            bool determine_resampling_necessary();

        protected:
            esiaf_ros::EsiafAudioTopicInfo topic;

            esiaf_ros::EsiafAudioFormat actualFormat;

            virtual void initialize_resampler() = 0;

            resampling::Resampler *resampler = nullptr;


        };

        /////////////////////////////////////////////////////////////////////////////////////
        // INPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        class InputTopicData : public TopicData {

        public:
            InputTopicData(ros::NodeHandle *nodeHandle,
                           esiaf_ros::EsiafAudioTopicInfo topic,
                           boost::function<void(const std::vector<int8_t> &,
                                                const esiaf_ros::RecordingTimeStamps &)> callback_ptr);

            void operator=(InputTopicData const &) = delete;  // delete the copy-assignment operator

        protected:
            void initialize_resampler();

        private:
            ros::Subscriber subscriber;

            boost::function<void(const std::vector<int8_t> &, const esiaf_ros::RecordingTimeStamps &)> userCallback;

            void internal_subscriber_callback(const esiaf_ros::AugmentedAudio::ConstPtr &msg);
        };


        /////////////////////////////////////////////////////////////////////////////////////
        // OUTPUT SUBCLASS
        ////////////////////////////////////////////////////////////////////////////////////
        class OutputTopicData : public TopicData {

        public:
            OutputTopicData(ros::NodeHandle *nodeHandle, esiaf_ros::EsiafAudioTopicInfo topic);

            void operator=(OutputTopicData const &) = delete;  // delete the copy-assignment operator

            void publish(std::vector<int8_t> signal, esiaf_ros::RecordingTimeStamps timeStamps);

        protected:
            void initialize_resampler();

        private:
            ros::Publisher publisher;
        };

    }
}

#endif //ESIAF_ROS_TOPICDATA_H
