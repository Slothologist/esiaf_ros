//
// Created by rfeldhans on 17.01.19.
//


#include "../include/resample.h"
#include <stdexcept>

namespace esiaf_ros {
    namespace resampling {


        Resampler::Resampler(esiaf_ros::EsiafAudioFormat inputFormat, esiaf_ros::EsiafAudioFormat outputFormat) {
            /*ROS_INFO("created resampler, which will resample from \n%d hz, %d bit, %d endian to\n%d hz, %d bit, %d endian",
                     inputFormat.rate, inputFormat.bitrate, inputFormat.endian,
                     outputFormat.rate, outputFormat.bitrate, outputFormat.endian);*/
            if (inputFormat.channels != outputFormat.channels) {
                std::string ex_text = "Different channel count is not supported for resampling yet";
                throw std::invalid_argument(ex_text);
            }
            switch (inputFormat.rate) {
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
            switch (outputFormat.rate) {
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
                    NULL,  // use defaults
                    NULL,  // use defaults
                    NULL); // use defaults
        }

        std::vector<int8_t> Resampler::resample(const std::vector<int8_t> &signal) {
            // prepare input
            size_t inputSize = signal.size();
            int8_t *inBuf = (int8_t *) signal.data();
            size_t *taken;

            // change bitrate before resampling if bitrate get converted up
            if (inputFormat.bitrate < outputFormat.bitrate) {
                int8_t *bitChangedBuf = (int8_t *) malloc(inputSize * bitrate_from_esiaf(outputFormat.bitrate));
                convert_bitrate(inBuf, inputSize, bitChangedBuf);
                inBuf = bitChangedBuf;
            }

            //prepare output
            auto outputSize = (size_t) ceil(inputSize * (orate / irate));
            auto outBuf = (int8_t *) malloc(outputSize);
            size_t *written;

            soxr_process(soxr, inBuf, inputSize, taken, outBuf, outputSize, written);

            // change bitrate after resampling if bitrate gets converted
            if (inputFormat.bitrate > outputFormat.bitrate) {
                int8_t *bitChangedBuf = (int8_t *) malloc(inputSize * bitrate_from_esiaf(outputFormat.bitrate));
                convert_bitrate(outBuf, inputSize, bitChangedBuf);
                free(outBuf);
                outBuf = bitChangedBuf;
            }

            // tidy up the malloc from before
            if (inputFormat.bitrate < outputFormat.bitrate) {
                free(inBuf);
            }

            // change input signal vector to resampled result
            std::vector<int8_t> newsignal(outBuf, outBuf + outputSize);
            return newsignal;
        }


        void Resampler::convert_bitrate_to_sox_sample_size(int8_t *framesIn,
                                                           size_t framesAmount,
                                                           sox_sample_t *framesOut) {

            sox_sample_t sox_macro_temp_sample;
            double sox_macro_temp_double;
            size_t clips = 0;

            for (int input = 0, output = 0; input < framesAmount; output++) {
                switch (inputFormat.bitrate) {
                    case esiaf_ros::Bitrate::BIT_INT_8_SIGNED:
                        framesOut[output] = SOX_SIGNED_8BIT_TO_SAMPLE(framesIn[input], clips);
                        input += 1;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED:
                        framesOut[output] = SOX_UNSIGNED_8BIT_TO_SAMPLE(framesIn[input], clips);
                        input += 1;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_16_SIGNED:
                        framesOut[output] = SOX_SIGNED_16BIT_TO_SAMPLE(framesIn[input], clips);
                        input += 2;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED:
                        framesOut[output] = SOX_UNSIGNED_16BIT_TO_SAMPLE(framesIn[input], clips);
                        input += 2;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_24_SIGNED:
                        framesOut[output] = SOX_SIGNED_24BIT_TO_SAMPLE(framesIn[input], clips);
                        input += 3;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED:
                        framesOut[output] = SOX_UNSIGNED_24BIT_TO_SAMPLE(framesIn[input], clips);
                        input += 3;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_32_SIGNED:
                        framesOut[output] = SOX_SIGNED_32BIT_TO_SAMPLE(framesIn[input], clips);
                        input += 4;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED:
                        framesOut[output] = SOX_UNSIGNED_32BIT_TO_SAMPLE(framesIn[input], clips);
                        input += 4;
                        break;
                    case esiaf_ros::Bitrate::BIT_FLOAT_32:
                        framesOut[output] = SOX_FLOAT_32BIT_TO_SAMPLE(framesIn[input], clips);
                        input += 4;
                        break;
                    case esiaf_ros::Bitrate::BIT_FLOAT_64:
                        framesOut[output] = SOX_FLOAT_64BIT_TO_SAMPLE(framesIn[input], clips);
                        input += 8;
                        break;
                    default:
                        std::string ex_text = "bitrate is not supported";
                        throw std::invalid_argument(ex_text);
                }
            }

        }

        void Resampler::convert_bitrate_from_sox_sample_size(sox_sample_t *framesIn,
                                                             size_t framesAmount,
                                                             int8_t *framesOut) {

            sox_sample_t sox_macro_temp_sample;
            double sox_macro_temp_double;
            size_t clips = 0;

            for (int input = 0, output = 0; output < framesAmount; input++) {
                switch (outputFormat.bitrate) {
                    case esiaf_ros::Bitrate::BIT_INT_8_SIGNED:
                        framesOut[output] = SOX_SAMPLE_TO_SIGNED_8BIT(framesIn[input], clips);
                        output += 1;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED:
                        framesOut[output] = SOX_SAMPLE_TO_UNSIGNED_8BIT(framesIn[input], clips);
                        output += 1;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_16_SIGNED:
                        framesOut[output] = SOX_SAMPLE_TO_SIGNED_16BIT(framesIn[input], clips);
                        output += 2;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED:
                        framesOut[output] = SOX_SAMPLE_TO_UNSIGNED_16BIT(framesIn[input], clips);
                        output += 2;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_24_SIGNED:
                        framesOut[output] = SOX_SAMPLE_TO_SIGNED_24BIT(framesIn[input], clips);
                        output += 3;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED:
                        framesOut[output] = SOX_SAMPLE_TO_UNSIGNED_24BIT(framesIn[input], clips);
                        output += 3;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_32_SIGNED:
                        framesOut[output] = SOX_SAMPLE_TO_SIGNED_32BIT(framesIn[input], clips);
                        output += 4;
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED:
                        framesOut[output] = SOX_SAMPLE_TO_UNSIGNED_32BIT(framesIn[input], clips);
                        output += 4;
                        break;
                    case esiaf_ros::Bitrate::BIT_FLOAT_32:
                        framesOut[output] = SOX_SAMPLE_TO_FLOAT_32BIT(framesIn[input], clips);
                        output += 4;
                        break;
                    case esiaf_ros::Bitrate::BIT_FLOAT_64:
                        framesOut[output] = SOX_SAMPLE_TO_FLOAT_64BIT(framesIn[input], clips);
                        output += 8;
                        break;
                    default:
                        std::string ex_text = "bitrate is not supported";
                        throw std::invalid_argument(ex_text);
                }
            }
        }

        void Resampler::convert_bitrate(int8_t *framesIn,
                                        size_t framesAmount,
                                        int8_t *framesOut) {
            // create buffer for intermediate buffer of type sox_sample_t
            sox_sample_t intermediateBuf[framesAmount];

            // do the actual conveting
            convert_bitrate_to_sox_sample_size(framesIn, framesAmount, &intermediateBuf[0]);
            convert_bitrate_from_sox_sample_size(&intermediateBuf[0], framesAmount, framesOut);
        }

        int Resampler::bitrate_from_esiaf(esiaf_ros::Bitrate bitrate) {
            switch (bitrate) {
                case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_8_SIGNED:
                    return 8;
                case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_16_SIGNED:
                    return 16;
                case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_24_SIGNED:
                    return 24;
                case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_32_SIGNED:
                case esiaf_ros::Bitrate::BIT_FLOAT_32:
                    return 32;
                case esiaf_ros::Bitrate::BIT_FLOAT_64:
                    return 64;
                default:
                    std::string ex_text = "bitrate is not supported";
                    throw std::invalid_argument(ex_text);
            }
        }


    }// namespace
}// namespace