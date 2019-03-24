//
// Created by rfeldhans on 09.03.19.
//
#include "../include/bitrateConvert.h"

namespace esiaf_ros {
    namespace converting {

        Converter::Converter(esiaf_ros::Bitrate inputBitrate, esiaf_ros::Bitrate outputBitrate) :
                inputBitrate(inputBitrate),
                outputBitrate(outputBitrate) {
            ROS_INFO("create converter to convert from %d bit to %d bit", (int)inputBitrate, (int)outputBitrate);
        }


        void Converter::convert_bitrate_to_sox_sample_size(int8_t *framesIn,
                                                           size_t framesAmount,
                                                           sox_sample_t *framesOut) {
            ROS_INFO("Converting bitrate taking place!");

            SOX_SAMPLE_LOCALS;
            size_t clips = 0;

            // prepare pointer casts
            int16_t* framesInCast_16 = (int16_t*) framesIn;
            sox_uint24_t* framesInCast_24 = (sox_uint24_t*) framesIn;
            int32_t* framesInCast_32 = (int32_t*) framesIn;
            double* framesInCast_64 = (double*) framesIn;

            for (int frame = 0; frame < framesAmount; frame++) {
                switch (inputBitrate) {
                    case esiaf_ros::Bitrate::BIT_INT_8_SIGNED:
                        framesOut[frame] = SOX_SIGNED_8BIT_TO_SAMPLE(framesIn[frame],);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED:
                        framesOut[frame] = SOX_UNSIGNED_8BIT_TO_SAMPLE(framesIn[frame],);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_16_SIGNED:
                        framesOut[frame] = SOX_SIGNED_16BIT_TO_SAMPLE(framesInCast_16[frame],);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED:
                        framesOut[frame] = SOX_UNSIGNED_16BIT_TO_SAMPLE(framesInCast_16[frame],);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_24_SIGNED:
                        framesOut[frame] = SOX_SIGNED_24BIT_TO_SAMPLE(framesInCast_24[frame],);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED:
                        framesOut[frame] = SOX_UNSIGNED_24BIT_TO_SAMPLE(framesInCast_24[frame], );
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_32_SIGNED:
                        framesOut[frame] = SOX_SIGNED_32BIT_TO_SAMPLE(framesInCast_32[frame],);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED:
                        framesOut[frame] = SOX_UNSIGNED_32BIT_TO_SAMPLE(framesInCast_32[frame],);
                        break;
                    case esiaf_ros::Bitrate::BIT_FLOAT_32:
                        framesOut[frame] = SOX_FLOAT_32BIT_TO_SAMPLE(framesInCast_32[frame], clips);
                        break;
                    case esiaf_ros::Bitrate::BIT_FLOAT_64:
                        framesOut[frame] = SOX_FLOAT_64BIT_TO_SAMPLE(framesInCast_64[frame], clips);
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

            SOX_SAMPLE_LOCALS;
            size_t clips = 0;

            // prepare pointer casts
            int16_t* framesOutCast_16 = (int16_t*) framesOut;
            sox_uint24_t* framesOutCast_24 = (sox_uint24_t*) framesOut;
            int32_t* framesOutCast_32 = (int32_t*) framesOut;
            double* framesOutCast_64 = (double*) framesOut;

            for (int frame = 0; frame < framesAmount; frame++) {
                
                switch (outputBitrate) {
                    case esiaf_ros::Bitrate::BIT_INT_8_SIGNED:
                        framesOut[frame] = SOX_SAMPLE_TO_SIGNED_8BIT(framesIn[frame], clips);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED:
                        framesOut[frame] = SOX_SAMPLE_TO_UNSIGNED_8BIT(framesIn[frame], clips);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_16_SIGNED:
                        framesOutCast_16[frame] = SOX_SAMPLE_TO_SIGNED_16BIT(framesIn[frame], clips);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED:
                        framesOutCast_16[frame] = SOX_SAMPLE_TO_UNSIGNED_16BIT(framesIn[frame], clips);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_24_SIGNED:
                        framesOutCast_24[frame] = SOX_SAMPLE_TO_SIGNED_24BIT(framesIn[frame], clips);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED:
                        framesOutCast_24[frame] = SOX_SAMPLE_TO_UNSIGNED_24BIT(framesIn[frame], clips);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_32_SIGNED:
                        framesOutCast_32[frame] = SOX_SAMPLE_TO_SIGNED_32BIT(framesIn[frame], clips);
                        break;
                    case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED:
                        framesOutCast_32[frame] = SOX_SAMPLE_TO_UNSIGNED_32BIT(framesIn[frame], clips);
                        break;
                    case esiaf_ros::Bitrate::BIT_FLOAT_32:
                        framesOutCast_32[frame] = SOX_SAMPLE_TO_FLOAT_32BIT(framesIn[frame], clips);
                        break;
                    case esiaf_ros::Bitrate::BIT_FLOAT_64:
                        framesOutCast_64[frame] = SOX_SAMPLE_TO_FLOAT_64BIT(framesIn[frame], clips);
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

        int byterate_from_esiaf(esiaf_ros::Bitrate bitrate) {
            switch (bitrate) {
                case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_8_SIGNED:
                    return sizeof(int8_t);
                case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_16_SIGNED:
                    return sizeof(int16_t);
                case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_24_SIGNED:
                    return 3;
                case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_32_SIGNED:
                case esiaf_ros::Bitrate::BIT_FLOAT_32:
                    return sizeof(int32_t);
                case esiaf_ros::Bitrate::BIT_FLOAT_64:
                    return sizeof(double);
                default:
                    std::string ex_text = "bitrate is not supported";
                    throw std::invalid_argument(ex_text);
            }
        }

    }
}
