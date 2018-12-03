//
// Created by rfeldhans on 04.11.18.
//
#include "../include/treegeneration.h"
#include <map>

namespace esiaf_ros {
    namespace treegeneration {

        void create_audio_tree(std::vector<nodestructures::EsiafNode> &nodes) {
            // create a hashmap of topics and their nodes
            std::map<std::string, std::vector<nodestructures::EsiafNode *>> inputTopics;
            std::map<std::string, nodestructures::EsiafNode *> outputTopics;

            // populate those hashmaps
            for (auto node : nodes) { // auto = nodestructures::EsiafNode
                // input topics
                for (auto inputTopicInfo : node.inputTopics) { // auto = EsiafAudioTopicInfo
                    inputTopics[inputTopicInfo.topic].push_back(&node);
                }
                // output topics
                for (auto outputTopicInfo : node.outputTopics) { // auto = EsiafAudioTopicInfo
                    if (outputTopics.find(outputTopicInfo.topic) != outputTopics.end())
                        outputTopics[outputTopicInfo.topic] = &node;
                    else // this would mean more than one node outputs on a specific topic, which could lead to problems in all nodes using this topic as input
                        ROS_ERROR("OutputTopic %s was defined twice, what is not allowed!", outputTopicInfo.topic.c_str());

                }
            }
            // populate the node connections
            for (auto node : nodes) { // auto = nodestructures::EsiafNode
                // populate input topics
                for (auto inputTopicInfo : node.inputTopics) { // auto = EsiafAudioTopicInfo
                    if (outputTopics.find(inputTopicInfo.topic) != outputTopics.end())
                        node.inputTopicNodes.push_back(&node);
                    else {
                        ROS_DEBUG("No node outputs on %s. How unfortunate.", inputTopicInfo.topic.c_str());
                    }
                }

                // populate output topics
                int topicCounter = 0; // will keep track of the topic which is processed
                for (auto outputTopicInfo : node.outputTopics) { // auto = EsiafAudioTopicInfo
                    if (inputTopics.find(outputTopicInfo.topic) != inputTopics.end())
                        node.outputTopicNodes[topicCounter].push_back(&node);
                    else {
                        ROS_DEBUG("No node reads on %s. But this is not our concern.", outputTopicInfo.topic.c_str());
                    }
                    topicCounter++;
                }
            }


        }
    }// namespace
}// namespace
