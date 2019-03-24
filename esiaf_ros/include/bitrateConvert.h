//
// Created by rfeldhans on 09.03.19.
//

#ifndef ESIAF_ROS_BITRATECONVERT_H
#define ESIAF_ROS_BITRATECONVERT_H

#include "esiaf_ros.h"
#include <soxr.h>
#include <sox.h>

namespace esiaf_ros {
    namespace converting {

        class Converter {
        public:
            Converter(esiaf_ros::Bitrate inputBitrate,
                      esiaf_ros::Bitrate outputBitrate);

            void convert_bitrate_to_sox_sample_size(int8_t *framesIn,
                                                    size_t framesAmount,
                                                    sox_sample_t *framesOut);

            void convert_bitrate_from_sox_sample_size(sox_sample_t *framesIn,
                                                      size_t framesAmount,
                                                      int8_t *framesOut);

            void convert_bitrate(int8_t *framesIn,
                                 size_t framesAmount,
                                 int8_t *framesOut);

        private:
            esiaf_ros::Bitrate inputBitrate;
            esiaf_ros::Bitrate outputBitrate;
        };

        int byterate_from_esiaf(esiaf_ros::Bitrate bitrate);

    }
}

#endif //ESIAF_ROS_BITRATECONVERT_H