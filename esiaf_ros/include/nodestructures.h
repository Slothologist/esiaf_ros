//
// Created by rfeldhans on 02.11.18.
//

#ifndef ESIAF_NODESTRUCTURES_H
#define ESIAF_NODESTRUCTURES_H

#include <string>
#include <vector>
#include "../include/esiaf_ros.h"

namespace esiaf_ros{
    namespace nodestructures{


        class EsiafNode{
        public:
            explicit EsiafNode(std::string name);

            // unique name of this node
            std::string name;

            // information about the input topics of this node
            std::vector<EsiafAudioTopicInfo> inputTopics;

            // the nodes which provide these topics
            std::vector<EsiafNode*> inputTopicNodes;

            // information about the output topics of this node
            std::vector<EsiafAudioTopicInfo> outputTopics;

            // the nodes which will use each of these topics (each topic can be used as input by several other nodes)
            std::vector<std::vector<EsiafNode*>> outputTopicNodes;

        };




    }// namespace
}// namespace

#endif //ESIAF_NODESTRUCTURES_H
