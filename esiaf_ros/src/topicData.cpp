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
                vadCallback_set(false),
                sslCallback_set(false),
                last_id(-1) {
            this->topic = topic;
            this->clientSideFormat = topic.allowedFormat;
            this->librarySideFormat = topic.allowedFormat;
            this->subscriber = nodeHandle->subscribe<esiaf_ros::AugmentedAudio>(topic.topic, 1000, boost::bind(
                    &InputTopicData::internal_subscriber_callback, this, _1));
            this->out_of_order_mutex = new std::mutex();
        }

        InputTopicData::~InputTopicData() {
            delete this->out_of_order_mutex;
        }

        void InputTopicData::internal_subscriber_callback(const esiaf_ros::AugmentedAudio::ConstPtr &msg) {
            ROS_DEBUG("msg id: %d; last id: %d" , msg->id, last_id);

            // check if the message is actually the next in line
            if(last_id == -1 || last_id > msg->id) { // first message || first message from new/ other publisher
                last_id = msg->id - 1;
            }

            bool skip_callbacks = false;

            if(msg->id > last_id+1){// message comes from the future
                std::lock_guard<std::mutex> lock(*out_of_order_mutex);
                out_of_order_msgs.push_back(msg);
                ROS_DEBUG("id: %d, lastid: %d, out_of_order %d", msg->id, last_id, out_of_order_msgs.size());
                if(out_of_order_msgs.size() > MAX_OUT_OF_ORDER_SIZE){
                    ROS_DEBUG("Max size reached");
                    // order size descending
                    auto orderfunction = [](const esiaf_ros::AugmentedAudio::ConstPtr &i1, const esiaf_ros::AugmentedAudio::ConstPtr &i2){
                        return i1->id > i2->id;
                    };
                    std::sort(out_of_order_msgs.begin(), out_of_order_msgs.end(), orderfunction);

                    // set last id to first id where all following ids are present
                    // purge all messages with previous ids
                    int new_last_id = out_of_order_msgs[0]->id;

                    for (auto it = out_of_order_msgs.begin()+1; it != out_of_order_msgs.end(); ) {
                        if((*it)->id == new_last_id - 1){
                            new_last_id = (*it)->id;
                            ++it;
                        } else{
                            it = out_of_order_msgs.erase(it);
                        }
                    }
                    last_id = new_last_id;
                    skip_callbacks = true;

                } else {
                    return;
                }
            }

            // call the actual client callbacks
            if(!skip_callbacks)
                call_client_callbacks(msg);

            // if we aquired out of order msgs, call the callback for these as well
            bool out_of_order_msgs_present = false;
            std::lock_guard<std::mutex> lock(*out_of_order_mutex);
            do{
                out_of_order_msgs_present = false;
                for (auto it = out_of_order_msgs.begin(); it != out_of_order_msgs.end(); ) {
                    if((*it)->id == last_id + 1){
                        out_of_order_msgs_present = true;

                        ROS_DEBUG("out of order msg coming through");
                        // call client callback function
                        call_client_callbacks(*it);

                        // remove msg from vector
                        it = out_of_order_msgs.erase(it);
                    } else {
                        ++it;
                    }
                }
            } while (out_of_order_msgs_present);
        }

        void InputTopicData::call_client_callbacks(const esiaf_ros::AugmentedAudio::ConstPtr &msg){
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
                if(sslCallback_set) {
                    sslCallback(msg->directions);
                }
            } catch (const std::exception &e) {
                ROS_INFO("%s", e.what());
            }

            try {
                if(msg->segmentation_ended && vadCallback_set) {
                    vadCallback();
                }
            } catch (const std::exception &e) {
                ROS_INFO("%s", e.what());
            }
            last_id++;
        }

        void InputTopicData::addVADcallback(boost::function<void()> callback_ptr) {
            ROS_INFO("adding vad callback");
            vadCallback = callback_ptr;
            vadCallback_set = true;
        }

        void InputTopicData::addSSLcallback(boost::function<void(
                const std::vector<esiaf_ros::SSLDir> &)> callback_ptr){
            ROS_INFO("adding vad callback");
            sslCallback = callback_ptr;
            vadCallback_set = true;
        }

        void InputTopicData::initialize_resampler() {
            this->resampler = new resampling::Resampler(this->librarySideFormat, this->clientSideFormat);
        }

        /////////////////////////////////////////////////////////////////////////////////////
        // OUTPUT SUBCLASS
        /////////////////////////////////////////////////////////////////////////////////////

        OutputTopicData::OutputTopicData(ros::NodeHandle *nodeHandle, esiaf_ros::EsiafAudioTopicInfo topic) :
                vadFinished(false),
                sslSet(false),
                current_id(1){

            this->topic = topic;
            this->librarySideFormat = topic.allowedFormat;
            this->clientSideFormat = topic.allowedFormat;
            this->publisher = nodeHandle->advertise<esiaf_ros::AugmentedAudio>(topic.topic, 1000);
            this->id_mutex = new std::mutex();
        }

        void OutputTopicData::publish(std::vector<int8_t> signal, esiaf_ros::RecordingTimeStamps timeStamps) {
            esiaf_ros::AugmentedAudio msg;
            {
                std::lock_guard<std::mutex> lock(*id_mutex);
                msg.id = current_id;
                current_id++;
            }
            std::vector<int8_t> signal_correctly_sampled;
            if (determine_resampling_necessary()) {
                ROS_DEBUG("output topicdata resampling call");
                signal_correctly_sampled = this->resampler->resample(signal);
            } else {
                signal_correctly_sampled = signal;
            }
            msg.signal = signal_correctly_sampled;
            msg.time = timeStamps;
            msg.segmentation_ended = vadFinished;
            if(sslSet){
                msg.directions = sslDirs;
            }
            this->publisher.publish(msg);
            ROS_DEBUG("output topicdata publish complete");
            vadFinished = false;
            sslSet = false;
        }

        void OutputTopicData::setVADfinished() {
            vadFinished = true;
        }

        void OutputTopicData::setSSLDirs(std::vector<esiaf_ros::SSLDir> sslDirs) {
            this->sslDirs = sslDirs;
            sslSet = true;
        }

        void OutputTopicData::initialize_resampler() {
            this->resampler = new resampling::Resampler(this->clientSideFormat, this->librarySideFormat);
        }
    }
}