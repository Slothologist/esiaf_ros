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
            esiaf_ros::EsiafAudioFormat inputFormat;
            esiaf_ros::EsiafAudioFormat outputFormat;

            sox_effects_chain_t * chain;

            double irate;
            double orate;

            int8_t* internal_input_buffer;
            size_t  internal_input_buffer_size;
            int8_t* internal_output_buffer;
            size_t  internal_output_buffer_size;

            sox_format_t  *in, *out;


            void setup_sox_effect_chain();
            void setup_sox_formats_from_esiaf();
            sox_encodinginfo_t sox_encodinginfo_from_esiaf(esiaf_ros::EsiafAudioFormat format);
            sox_signalinfo_t sox_signalinfo_from_esiaf(esiaf_ros::EsiafAudioFormat format);
            double sox_rate_from_esiaf(esiaf_ros::Rate rate);
        };


    }// namespace
}// namespace

#endif //ESIAF_ROS_RESAMPLE_H
