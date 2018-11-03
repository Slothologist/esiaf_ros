//
// Created by rfeldhans on 02.11.18.
//

#ifndef ESIAF_NODESTRUCTURES_H
#define ESIAF_NODESTRUCTURES_H

#include <string>

namespace esiaf{
    namespace nodestructures{

        struct AudioFormat{
            int rate;
            int bitrate;
            std::string endian;
        };

        struct AudioInfoTopic{
            std::string topic;
            AudioFormat allowedFormats[];
        };

        class EsiafNode{
        public:
            explicit EsiafNode();

        private:

        };


    }
}

#endif //ESIAF_NODESTRUCTURES_H
