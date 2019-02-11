//
// Created by rfeldhans on 17.01.19.
//

#ifndef ESIAF_ROS_RESAMPLE_H
#define ESIAF_ROS_RESAMPLE_H

#include "esiaf_ros.h"
#include <soxr.h>
#include <vector>

namespace esiaf_ros {
    namespace resampling {

        class Resampler {
        public:
            Resampler(esiaf_ros::EsiafAudioFormat inputFormat,
                      esiaf_ros::EsiafAudioFormat outputFormat);

            void resample(std::vector<int8_t>* signal);

        protected:
            esiaf_ros::EsiafAudioFormat inputFormat;
            esiaf_ros::EsiafAudioFormat outputFormat;

        private:
            soxr_t soxr;
            double irate;
            double orate;

        };


    }// namespace
}// namespace

#endif //ESIAF_ROS_RESAMPLE_H
