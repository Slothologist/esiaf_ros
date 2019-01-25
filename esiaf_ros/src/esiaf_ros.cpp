//
// Created by rfeldhans on 02.11.18.
//

// project includes
#include "../include/esiaf_ros.h"
#include "../include/topicData.h"
#include "esiaf_ros/NodeInfo.h"

// ros msg includes
#include "esiaf_ros/RegisterNode.h"
#include "esiaf_ros/ChangedConfig.h"
#include "esiaf_ros/RecordingTimeStamps.h"
#include "esiaf_ros/AudioTopicInfo.h"

// std includes
#include <vector>
#include <string>
#include <unistd.h>


namespace esiaf_ros {
    using namespace esiaf_ros;

    std::string nodename;
    NodeDesignation nodedesignation;

    // library data structures
    std::vector<topicdata::InputTopicData> inputs;
    std::vector<topicdata::OutputTopicData> outputs;

    // ros publisher and subscriber
    ros::NodeHandle *ros_node_handle;
    ros::Subscriber changeConfigSub;
    ros::ServiceClient registerService;


    void handle_changed_config(const esiaf_ros::ChangedConfig::ConstPtr& msg){
        for(auto input : inputs){
            for(auto changedInput: msg->inputTopics){
                if(input.getTopicName() == changedInput.topic){
                    input.setActualFormat(changedInput.allowedFormat);
                    break;
                }
            }
        }
        for(auto output : outputs){
            for(auto changedOutput: msg->outputTopics){
                if(output.getTopicName() == changedOutput.topic){
                    output.setActualFormat(changedOutput.allowedFormat);
                    break;
                }
            }
        }
    }


    void initialize_esiaf(ros::NodeHandle *nodeHandle, NodeDesignation nodeDesignation) {
        // initialize basic stuff
        ros_node_handle = nodeHandle;
        nodename = ros::this_node::getName();
        nodedesignation = nodeDesignation;

        // create registration publisher and config subscriber
        changeConfigSub = ros_node_handle->subscribe(std::string("/esiaf_ros/") + nodename + std::string("/changedConfig"), 1000, handle_changed_config);
        registerService = ros_node_handle->serviceClient<esiaf_ros::RegisterNode>("/esiaf_ros/orchestrator/register");
    }

    void add_input_topic(EsiafAudioTopicInfo &input,
                         const std::function<void(std::vector<int8_t>, esiaf_ros::RecordingTimeStamps)>& callback) {
        inputs.push_back(topicdata::InputTopicData(ros_node_handle, input, callback));
    }

    void add_output_topic(EsiafAudioTopicInfo &output) {
        outputs.push_back(topicdata::OutputTopicData(ros_node_handle, output));
    }


    void start_esiaf() {
        ROS_INFO("esaif_lib: starting esiaf");
        // create RegisterNode msg and publish it
        esiaf_ros::RegisterNode registerNode;
        registerNode.request.name = nodename;
        registerNode.request.designation = (char) nodedesignation;
        for(auto input : inputs){// auto = InputTopicData
            esiaf_ros::AudioTopicInfo info;
            info.topic = input.getTopicName();

            info.allowedFormat.rate = (int)input.getInfo().allowedFormat.rate;
            info.allowedFormat.bitrate = (int)input.getInfo().allowedFormat.bitrate;
            info.allowedFormat.channels = input.getInfo().allowedFormat.channels;
            info.allowedFormat.endian = (int)input.getInfo().allowedFormat.endian;

            registerNode.request.inputTopics.push_back(info);
        }

        for(auto output : outputs){// auto = OutputTopicData
            esiaf_ros::AudioTopicInfo info;
            info.topic = output.getTopicName();

            info.allowedFormat.rate = (int)output.getInfo().allowedFormat.rate;
            info.allowedFormat.bitrate = (int)output.getInfo().allowedFormat.bitrate;
            info.allowedFormat.channels = output.getInfo().allowedFormat.channels;
            info.allowedFormat.endian = (int)output.getInfo().allowedFormat.endian;

            registerNode.request.outputTopics.push_back(info);
        }

        ROS_INFO("esiaf_lib: publishing registernode");
        ros::service::waitForService("/esiaf_ros/orchestrator/register", 15000);
        if(!registerService.call(registerNode)){
            ROS_ERROR("Failed to register node!");
        }
    }


    void publish(std::string topic, std::vector<int8_t> signal, esiaf_ros::RecordingTimeStamps timeStamps) {
        for (auto outputTopic : outputs) {// auto = OutputTopicData
            if(outputTopic.getTopicName() == topic){
                outputTopic.publish(signal, timeStamps);
            }
            return;
        }
        // raise exception when topic name could not be matched
    }

}// namespace