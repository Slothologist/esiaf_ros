//
// Created by rfeldhans on 09.03.19.
//
#include "../include/bitrateConvert.h"

namespace esiaf_ros {
    namespace converting {

        Converter::Converter(esiaf_ros::EsiafAudioFormat inputFormat, esiaf_ros::EsiafAudioFormat outputFormat) :
                inputFormat(inputFormat),
                outputFormat(outputFormat) {

        }


        void Converter::convert_bitrate_to_sox_sample_size(int8_t *framesIn,
                                                           size_t framesAmount,
                                                           sox_sample_t *framesOut) {
            ROS_INFO("Converting bitrate taking place!");

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

        void Converter::convert_bitrate_from_sox_sample_size(sox_sample_t *framesIn,
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

        void Converter::convert_bitrate(int8_t *framesIn,
                                        size_t framesAmount,
                                        int8_t *framesOut) {
            // create buffer for intermediate buffer of type sox_sample_t
            sox_sample_t intermediateBuf[framesAmount];

            // do the actual conveting
            convert_bitrate_to_sox_sample_size(framesIn, framesAmount, &intermediateBuf[0]);
            convert_bitrate_from_sox_sample_size(&intermediateBuf[0], framesAmount, framesOut);
        }

        int bitrate_from_esiaf(esiaf_ros::Bitrate bitrate) {
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

    }
}
