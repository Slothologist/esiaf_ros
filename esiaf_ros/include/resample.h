//
// Created by rfeldhans on 17.01.19.
//

#ifndef ESIAF_ROS_RESAMPLE_H
#define ESIAF_ROS_RESAMPLE_H

#include "esiaf_ros.h"
#include "bitrateConvert.h"
#include <soxr.h>
#include <sox.h>
#include <vector>

#define soxr_bitrate esiaf_ros::Bitrate::BIT_INT_32_SIGNED

namespace esiaf_ros {
    namespace resampling {

        class Resampler {
        public:
            Resampler(esiaf_ros::EsiafAudioFormat inputFormat,
                      esiaf_ros::EsiafAudioFormat outputFormat);

            std::vector<int8_t> resample(const std::vector<int8_t> &signal);

        private:
            esiaf_ros::EsiafAudioFormat inputFormat;
            esiaf_ros::EsiafAudioFormat outputFormat;

            esiaf_ros::converting::Converter converter_to_soxr_sample_size;
            esiaf_ros::converting::Converter converter_from_soxr_sample_size;

            soxr_t soxr;
            double irate;
            double orate;

        };


    }// namespace
}// namespace

#endif //ESIAF_ROS_RESAMPLE_H
