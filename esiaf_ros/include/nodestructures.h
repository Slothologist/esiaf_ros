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

            std::string name;
            std::vector<EsiafAudioTopicInfo> inputTopics;
            std::vector<EsiafAudioTopicInfo> outputTopics;
        };




    }// namespace
}// namespace

#endif //ESIAF_NODESTRUCTURES_H
