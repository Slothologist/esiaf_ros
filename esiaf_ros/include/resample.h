//
// Created by rfeldhans on 17.01.19.
//

#ifndef ESIAF_ROS_RESAMPLE_H
#define ESIAF_ROS_RESAMPLE_H

#include "esiaf_ros.h"
#include <soxr.h>
#include <sox.h>
#include <vector>

namespace esiaf_ros {
    namespace resampling {

        class Resampler {
        public:
            Resampler(esiaf_ros::EsiafAudioFormat inputFormat,
                      esiaf_ros::EsiafAudioFormat outputFormat);

            std::vector<int8_t> resample(const std::vector<int8_t> &signal);

        protected:
            esiaf_ros::EsiafAudioFormat inputFormat;
            esiaf_ros::EsiafAudioFormat outputFormat;

            void convert_bitrate_to_sox_sample_size(int8_t *framesIn,
                                                    size_t framesAmount,
                                                    sox_sample_t *framesOut);

            void convert_bitrate_from_sox_sample_size(sox_sample_t *framesIn,
                                                      size_t framesAmount,
                                                      int8_t *framesOut);

            void convert_bitrate(int8_t *framesIn,
                                 size_t framesAmount,
                                 int8_t *framesOut);

            int bitrate_from_esiaf(esiaf_ros::Bitrate bitrate);

        private:
            soxr_t soxr;
            double irate;
            double orate;

        };


    }// namespace
}// namespace

#endif //ESIAF_ROS_RESAMPLE_H
