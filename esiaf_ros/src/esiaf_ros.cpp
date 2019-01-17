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
    ros::Publisher registerPub;


    void handle_changed_config(const esiaf_ros::ChangedConfig::ConstPtr& msg){

    }


    void initialize_esiaf(ros::NodeHandle *nodeHandle, NodeDesignation nodeDesignation) {
        // initialize basic stuff
        ros_node_handle = nodeHandle;
        nodename = ros::this_node::getName();
        nodedesignation = nodeDesignation;

        // create registration publisher and config subscriber
        changeConfigSub = ros_node_handle->subscribe(std::string("/esiaf_ros/") + nodename + std::string("/changedConfig"), 1000, handle_changed_config);
        registerPub = ros_node_handle->advertise<esiaf_ros::RegisterNode>("/esiaf_ros/orchestrator/register", 1000);
    }

    // TODO: add callback function
    void add_input_topic(EsiafAudioTopicInfo &input, std::function<void(char*, size_t, esiaf_ros::RecordingTimeStamps)> callback) {
        using namespace topicdata;
        InputTopicData outputTopic = InputTopicData(ros_node_handle, input, callback);
    }

    void add_output_topic(EsiafAudioTopicInfo &output) {
        using namespace topicdata;
        OutputTopicData outputTopic = OutputTopicData(ros_node_handle, output);
    }


    void start_esiaf() {
        // create RegisterNode msg and publish it
        esiaf_ros::RegisterNode registerNode;
        registerNode.name = nodename;
        registerNode.designation = (char) nodedesignation;


        registerPub.publish(registerNode);
    }


    void publish(std::string topic, char *signalBuffer, size_t buffersize, esiaf_ros::RecordingTimeStamps timeStamps) {
        for (auto outputTopic : outputs) {// auto = OutputTopicData
            if(outputTopic.getTopicName() == topic){
                outputTopic.publish(signalBuffer, buffersize, timeStamps);
            }
            return;
        }
        // raise exception when topic name could not be matched
    }

}// namespace