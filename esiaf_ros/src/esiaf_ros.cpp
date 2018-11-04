//
// Created by rfeldhans on 02.11.18.
//

// project includes
#include "../include/esiaf_ros.h"
#include "../include/nodestructures.h"
#include "../include/treegeneration.h"
#include "esiaf_ros/NodeInfo.h"

// std includes
#include <vector>
#include <string>
#include <unistd.h>

namespace esiaf_ros{

    // library specific parameters
    std::string esiaf_topic = "/esiaf/node_info_topic";
    std::string preferred_audio_rate = 0;

    // library data structures
    std::vector<nodestructures::EsiafNode> nodes;
    nodestructures::EsiafNode own_representation(0);

    // ros publisher and subscriber
    ros::NodeHandle* ros_node_handle;
    ros::Publisher publisher;
    ros::Subscriber subscriber;

    // state keeping variables
    bool started = false;


    void publish_own_info(){
        esiaf_ros::NodeInfo nodeinfo;

        nodeinfo.name = own_representation.name;

        // converting input topics
        std::vector<EsiafAudioTopicInfo>::iterator it;
        for(it = own_representation.inputTopics.begin(); it != own_representation.inputTopics.end(); it++)    {
            esiaf_ros::AudioTopicInfo topicinfo;

            topicinfo.topic = it->topic;
            std::vector<EsiafAudioFormat>::iterator it2;
            for(it2 = it->allowedFormats.begin(); it2 != it->allowedFormats.end(); it2++) {
                esiaf_ros::AudioFormat format;

                format.bitrate = it2->bitrate;
                format.channels = it2->channels;
                format.rate = it2->rate;
                format.endian = it2->endian;

                topicinfo.allowedFormats.push_back(format);
            }

            nodeinfo.inputTopics.push_back(topicinfo);
        }
        // basically the same for output topics
        for(it = own_representation.outputTopics.begin(); it != own_representation.outputTopics.end(); it++)    {
            esiaf_ros::AudioTopicInfo topicinfo;

            topicinfo.topic = it->topic;
            std::vector<EsiafAudioFormat>::iterator it2;
            for(it2 = it->allowedFormats.begin(); it2 != it->allowedFormats.end(); it2++) {
                esiaf_ros::AudioFormat format;

                format.bitrate = it2->bitrate;
                format.channels = it2->channels;
                format.rate = it2->rate;
                format.endian = it2->endian;

                topicinfo.allowedFormats.push_back(format);
            }

            nodeinfo.outputTopics.push_back(topicinfo);
        }

        // dont forget to publish
        publisher.publish(nodeinfo);
    }

    nodestructures::EsiafNode create_EsiafNode_from_msg(esiaf_ros::NodeInfo &nodeInfo){
        nodestructures::EsiafNode node(nodeInfo.name);

        // converting input topics
        std::vector<esiaf_ros::AudioTopicInfo>::iterator it;
        for(it = nodeInfo.inputTopics.begin(); it != nodeInfo.inputTopics.end(); it++)    {
            EsiafAudioTopicInfo topicinfo;

            topicinfo.topic = it->topic;
            std::vector<AudioFormat>::iterator it2;
            for(it2 = it->allowedFormats.begin(); it2 != it->allowedFormats.end(); it2++) {
                EsiafAudioFormat format;

                format.bitrate = it2->bitrate;
                format.channels = it2->channels;
                format.rate = it2->rate;
                format.endian = it2->endian;

                topicinfo.allowedFormats.push_back(format);
            }

            node.inputTopics.push_back(topicinfo);
        }
        // basically the same for output topics
        for(it = nodeInfo.outputTopics.begin(); it != nodeInfo.outputTopics.end(); it++)    {
            EsiafAudioTopicInfo topicinfo;

            topicinfo.topic = it->topic;
            std::vector<AudioFormat>::iterator it2;
            for(it2 = it->allowedFormats.begin(); it2 != it->allowedFormats.end(); it2++) {
                EsiafAudioFormat format;

                format.bitrate = it2->bitrate;
                format.channels = it2->channels;
                format.rate = it2->rate;
                format.endian = it2->endian;

                topicinfo.allowedFormats.push_back(format);
            }

            node.outputTopics.push_back(topicinfo);
        }

        return node;
    }

    void subscriber_callback(esiaf_ros::NodeInfo msg){

        bool node_known = false;

        // iterate through known nodes
        std::vector<nodestructures::EsiafNode>::iterator it;
        for(it = nodes.begin(); it != nodes.end(); it++)    {
            if(it->name == msg.name) {
                node_known = true;
                if(!msg.alive){ // node shuts down, remove node from internal node vector
                    nodes.erase(it);
                    // may need to rebuild the audio tree, if only the lib was
                    break;
                } else{
                    // node is already known, no need to do anything
                    break;
                }
            }
        }
        // if the node sending the message wasn't known, add it to the internal node vector and send our own
        if(! node_known){
            nodes.push_back(create_EsiafNode_from_msg(msg));
            if(started) {
                publish_own_info();
            }
        }
    }

    void initialize_esiaf(ros::NodeHandle* nodeHandle){
        // initialize basic stuff
        ros_node_handle = nodeHandle;
        own_representation = nodestructures::EsiafNode(ros::this_node::getName());
        publisher = ros_node_handle->advertise<esiaf_ros::NodeInfo>(esiaf_topic, 1);
        subscriber = ros_node_handle->subscribe<esiaf_ros::NodeInfo>(esiaf_topic, 1000, subscriber_callback);
        ros::param::get("/esiaf/preferred_audio_rate", preferred_audio_rate);
    }

    // TODO: add callback function
    void add_input_topic(EsiafAudioTopicInfo &input){
        own_representation.inputTopics.push_back(input);
    }

    // TODO: add function to call to output audio?
    void add_output_topic(EsiafAudioTopicInfo &output){
        own_representation.outputTopics.push_back(output);
    }


    void start_esiaf(){
        // make yourself known to other esiaf nodes
        started = true;
        publish_own_info();

        // wait a bit for other nodes to respond
        usleep(1000 * 500); // 500 milliseconds

        // create an audio tree
        esiaf_ros::treegeneration::create_audio_tree();
    }

}// namespace