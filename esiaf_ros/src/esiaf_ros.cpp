//
// Created by rfeldhans on 02.11.18.
//

// std includes
#include <stdexcept>

// project includes
#include "../include/esiaf_ros.h"
#include "../include/topicData.h"

// ros msg includes
#include "esiaf_ros/AudioTopicInfo.h"
#include "esiaf_ros/NodeInfo.h"
#include "esiaf_ros/RegisterNode.h"
#include "esiaf_ros/ChangedConfig.h"
#include "esiaf_ros/RecordingTimeStamps.h"


namespace esiaf_ros {
    using namespace esiaf_ros;


    struct esiaf_handle{
        std::string nodename;
        NodeDesignation nodedesignation;

        // library data structures
        std::vector<topicdata::InputTopicData> inputs;
        std::vector<topicdata::OutputTopicData> outputs;

        // ros publisher and subscriber
        ros::NodeHandle *ros_node_handle;
        ros::Subscriber changeConfigSub;
        ros::ServiceClient registerService;
    };



    void handle_changed_config(esiaf_handle* esiafHandle,
                               const esiaf_ros::ChangedConfig& msg){
        ROS_DEBUG("Received request to change config.");
        for(auto&& input : esiafHandle->inputs){ // auto = topicdata::InputTopicData
            for(auto changedInput: msg.inputTopics){
                if(input.getTopicName() == changedInput.topic){
                    input.setLibrarySideFormat(changedInput.allowedFormat);
                    break;
                }
            }
        }
        for(auto&& output : esiafHandle->outputs){ // auto = topicdata::OutputTopicData
            for(auto changedOutput: msg.outputTopics){
                if(output.getTopicName() == changedOutput.topic){
                    output.setLibrarySideFormat(changedOutput.allowedFormat);
                    break;
                }
            }
        }
    }

    Esiaf_Handler::Esiaf_Handler(ros::NodeHandle *nodeHandle,
                                 NodeDesignation nodeDesignation) {
        // initialize basic stuff
        handle = new esiaf_handle;
        handle->ros_node_handle = nodeHandle;
        handle->nodename = ros::this_node::getName();
        handle->nodedesignation = nodeDesignation;


        boost::function<void(const esiaf_ros::ChangedConfig&)> config_change_function = [&](const esiaf_ros::ChangedConfig& msg){
            handle_changed_config(handle, msg);
        };

        // create registration publisher and config subscriber
        handle->changeConfigSub = handle->ros_node_handle->subscribe<esiaf_ros::ChangedConfig>(
                std::string("/esiaf_ros/") + handle->nodename + std::string("/changedConfig"),
                1000,
                config_change_function);
        handle->registerService = handle->ros_node_handle->serviceClient<esiaf_ros::RegisterNode>("/esiaf_ros/orchestrator/register");
    }

    void Esiaf_Handler::add_input_topic(EsiafAudioTopicInfo &input,
                                        boost::function<void(
            const std::vector<int8_t> &,
            const esiaf_ros::RecordingTimeStamps &)> callback) {
        handle->inputs.emplace_back(handle->ros_node_handle, input, callback);
        std::cout << "add_input_topic in esiaf handler cpp finished" << std::endl << std::flush;
    }

    void Esiaf_Handler::add_output_topic(EsiafAudioTopicInfo &output) {
        handle->outputs.emplace_back(handle->ros_node_handle, output);
    }

    void Esiaf_Handler::start_esiaf() {
        ROS_DEBUG("esaif_lib: starting esiaf");
        // create RegisterNode msg and publish it
        esiaf_ros::RegisterNode registerNode;
        registerNode.request.name = handle->nodename;
        registerNode.request.designation = (char)handle->nodedesignation;
        for(auto&& input : handle->inputs){// auto = InputTopicData
            esiaf_ros::AudioTopicInfo info;
            info.topic = input.getTopicName();

            info.allowedFormat.rate = (int)input.getInfo().allowedFormat.rate;
            info.allowedFormat.bitrate = (int)input.getInfo().allowedFormat.bitrate;
            info.allowedFormat.channels = input.getInfo().allowedFormat.channels;
            info.allowedFormat.endian = (int)input.getInfo().allowedFormat.endian;

            registerNode.request.inputTopics.push_back(info);
        }

        for(auto&& output : handle->outputs){// auto = OutputTopicData
            esiaf_ros::AudioTopicInfo info;
            info.topic = output.getTopicName();

            info.allowedFormat.rate = (int)output.getInfo().allowedFormat.rate;
            info.allowedFormat.bitrate = (int)output.getInfo().allowedFormat.bitrate;
            info.allowedFormat.channels = output.getInfo().allowedFormat.channels;
            info.allowedFormat.endian = (int)output.getInfo().allowedFormat.endian;

            registerNode.request.outputTopics.push_back(info);
        }

        ROS_DEBUG("esiaf_lib: publishing registernode");
        ros::service::waitForService("/esiaf_ros/orchestrator/register", 15000);
        if(!handle->registerService.call(registerNode)){
            ROS_ERROR("esiaf_ros: Failed to register node!");
        }
    }

    void Esiaf_Handler::publish(std::string topic,
                                std::vector<int8_t> signal,
                                esiaf_ros::RecordingTimeStamps timeStamps) {
        for (auto&& outputTopic : handle->outputs) {// auto = OutputTopicData
            if(outputTopic.getTopicName() == topic){
                outputTopic.publish(signal, timeStamps);
            }
            return;
        }
        // raise exception when topic name could not be matched
    }

    void Esiaf_Handler::add_vad_finished_callback(EsiafAudioTopicInfo &input,
                                                  boost::function<void()> callback) {
        for (auto&& inputTopic : handle->inputs) {// auto = InputTopicData
            if(inputTopic.getTopicName() == input.topic){
                inputTopic.addVADcallback(callback);
                return;
            }
        }
        // raise exception?
    }

    void Esiaf_Handler::set_vad_finished(std::string topic) {
        for (auto&& outputTopic : handle->outputs) {// auto = OutputTopicData
            if(outputTopic.getTopicName() == topic){
                outputTopic.setVADfinished();
            }
            return;
        }
    }

    void Esiaf_Handler::set_ssl_dirs(std::string topic, std::vector<esiaf_ros::SSLDir> sslDirs) {
        for (auto&& outputTopic : handle->outputs) {// auto = OutputTopicData
            if(outputTopic.getTopicName() == topic){
                outputTopic.setSSLDirs(sslDirs);
            }
            return;
        }
    }

    void Esiaf_Handler::add_ssl_dir_callback(EsiafAudioTopicInfo &input,
                                             boost::function<void(
                                                     const std::vector<esiaf_ros::SSLDir>&
                                             )> callback) {
        for (auto&& inputTopic : handle->inputs) {// auto = InputTopicData
            if(inputTopic.getTopicName() == input.topic){
                inputTopic.addSSLcallback(callback);
                return;
            }
        }
        // raise exception?
    }

    void Esiaf_Handler::quit_esiaf() {
        delete handle;
    }

}// namespace