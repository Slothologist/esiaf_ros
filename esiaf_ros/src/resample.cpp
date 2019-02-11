//
// Created by rfeldhans on 17.01.19.
//


#include "../include/resample.h"
#include <stdexcept>

namespace esiaf_ros {
    namespace resampling {


        Resampler::Resampler(esiaf_ros::EsiafAudioFormat inputFormat, esiaf_ros::EsiafAudioFormat outputFormat) {
            if (inputFormat.channels != outputFormat.channels){
                std::string ex_text = "Different channel count is not supported for resampling yet";
                throw std::invalid_argument(ex_text);
            }
            switch (inputFormat.rate){
                case esiaf_ros::Rate::RATE_8000:
                    irate = 8000.;
                    break;
                case esiaf_ros::Rate::RATE_16000:
                    irate = 16000.;
                    break;
                case esiaf_ros::Rate::RATE_32000:
                    irate = 32000.;
                    break;
                case esiaf_ros::Rate::RATE_44100:
                    irate = 44100.;
                    break;
                case esiaf_ros::Rate::RATE_48000:
                    irate = 48000.;
                    break;
                case esiaf_ros::Rate::RATE_96000:
                    irate = 96000.;
                    break;
                default:
                    std::string ex_text = "input sample rate is not supported";
                    throw std::invalid_argument(ex_text);

            }
            switch (outputFormat.rate){
                case esiaf_ros::Rate::RATE_8000:
                    orate = 8000.;
                    break;
                case esiaf_ros::Rate::RATE_16000:
                    orate = 16000.;
                    break;
                case esiaf_ros::Rate::RATE_32000:
                    orate = 32000.;
                    break;
                case esiaf_ros::Rate::RATE_44100:
                    orate = 44100.;
                    break;
                case esiaf_ros::Rate::RATE_48000:
                    orate = 48000.;
                    break;
                case esiaf_ros::Rate::RATE_96000:
                    orate = 96000.;
                    break;
                default:
                    std::string ex_text = "input sample rate is not supported";
                    throw std::invalid_argument(ex_text);

            }

            soxr_error_t error;

            soxr = soxr_create(
                    irate, orate, inputFormat.channels,             /* Input rate, output rate, # of channels. */
                    &error,                         /* To report any error during creation. */
                    NULL,  // create based on formats //TODO
                    NULL,  // use defaults
                    NULL); // use defaults
        }

        void Resampler::resample(std::vector<int8_t> *signal) {
            // prepare input
            size_t inputSize = signal->size();
            auto inBuf = signal->data();
            size_t* taken;

            //prepare output
            auto outputSize = (size_t) ceil(signal->size() * (orate/irate));
            auto outBuf = (int8_t*) malloc(outputSize);
            size_t* written;

            soxr_process(soxr, inBuf, inputSize, taken, outBuf, outputSize, written);

            // change input signal vector to resampled result
            //TODO
        }


    }// namespace
}// namespace