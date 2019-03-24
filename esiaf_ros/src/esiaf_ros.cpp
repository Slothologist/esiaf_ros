//
// Created by rfeldhans on 02.11.18.
//

// std includes
#include <sox.h>
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


    esiaf_handle* initialize_esiaf(ros::NodeHandle *nodeHandle,
                          NodeDesignation nodeDesignation) {
        // initialize basic stuff
        static esiaf_handle* esiafHandle = new esiaf_handle;
        esiafHandle->ros_node_handle = nodeHandle;
        esiafHandle->nodename = ros::this_node::getName();
        esiafHandle->nodedesignation = nodeDesignation;


        boost::function<void(const esiaf_ros::ChangedConfig&)> config_change_function = [&](const esiaf_ros::ChangedConfig& msg){
            handle_changed_config(esiafHandle, msg);
        };

        // create registration publisher and config subscriber
        esiafHandle->changeConfigSub = esiafHandle->ros_node_handle->subscribe<esiaf_ros::ChangedConfig>(
                std::string("/esiaf_ros/") + esiafHandle->nodename + std::string("/changedConfig"),
                1000,
                config_change_function);
        esiafHandle->registerService = esiafHandle->ros_node_handle->serviceClient<esiaf_ros::RegisterNode>("/esiaf_ros/orchestrator/register");
        return esiafHandle;
    }

    void add_input_topic(esiaf_handle* esiafHandle,
                         EsiafAudioTopicInfo &input,
                         boost::function<void(
                                 const std::vector<int8_t>&,
                                 const esiaf_ros::RecordingTimeStamps&)> callback) {
        esiafHandle->inputs.emplace_back(esiafHandle->ros_node_handle, input, callback);
    }

    void add_output_topic(esiaf_handle* esiafHandle,
                          EsiafAudioTopicInfo &output) {
        esiafHandle->outputs.emplace_back(esiafHandle->ros_node_handle, output);
    }


    void start_esiaf(esiaf_handle* esiafHandle) {
        ROS_DEBUG("esaif_lib: starting esiaf");
        // create RegisterNode msg and publish it
        esiaf_ros::RegisterNode registerNode;
        registerNode.request.name = esiafHandle->nodename;
        registerNode.request.designation = (char)esiafHandle->nodedesignation;
        for(auto&& input : esiafHandle->inputs){// auto = InputTopicData
            esiaf_ros::AudioTopicInfo info;
            info.topic = input.getTopicName();

            info.allowedFormat.rate = (int)input.getInfo().allowedFormat.rate;
            info.allowedFormat.bitrate = (int)input.getInfo().allowedFormat.bitrate;
            info.allowedFormat.channels = input.getInfo().allowedFormat.channels;
            info.allowedFormat.endian = (int)input.getInfo().allowedFormat.endian;

            registerNode.request.inputTopics.push_back(info);
        }

        for(auto&& output : esiafHandle->outputs){// auto = OutputTopicData
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
        if(!esiafHandle->registerService.call(registerNode)){
            ROS_ERROR("esiaf_ros: Failed to register node!");
        }
    }


    void publish(esiaf_handle* esiafHandle,
                 std::string topic,
                 std::vector<int8_t> signal,
                 esiaf_ros::RecordingTimeStamps timeStamps) {
        for (auto&& outputTopic : esiafHandle->outputs) {// auto = OutputTopicData
            if(outputTopic.getTopicName() == topic){
                outputTopic.publish(signal, timeStamps);
            }
            return;
        }
        // raise exception when topic name could not be matched
    }

    void add_vad_finished_callback(esiaf_handle* esiafHandle,
                                   std::string topic,
                                   boost::function<void()> callback){
        for (auto&& inputTopic : esiafHandle->inputs) {// auto = InputTopicData
            if(inputTopic.getTopicName() == topic){
                inputTopic.addVADcallback(callback);
            }
            return;
        }
    }

    void set_vad_finished(esiaf_handle* esiafHandle,
                          std::string topic){
        for (auto&& outputTopic : esiafHandle->outputs) {// auto = OutputTopicData
            if(outputTopic.getTopicName() == topic){
                outputTopic.setVADfinished();
            }
            return;
        }
    }

}// namespace