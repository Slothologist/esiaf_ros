//
// Created by rfeldhans on 04.11.18.
//

#ifndef ESIAF_TREEGENERATION_H
#define ESIAF_TREEGENERATION_H

#include <vector>
#include "../include/nodestructures.h"

namespace esiaf_ros {
    namespace treegeneration {

        void create_audio_tree(std::vector<nodestructures::EsiafNode> &nodes);
    }
}

#endif //ESIAF_TREEGENERATION_H
