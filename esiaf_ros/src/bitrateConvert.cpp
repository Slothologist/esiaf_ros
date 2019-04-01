//
// Created by rfeldhans on 09.03.19.
//
#include "../include/bitrateConvert.h"

namespace esiaf_ros {
    namespace converting {

        Converter::Converter(esiaf_ros::Bitrate inputBitrate, esiaf_ros::Bitrate outputBitrate) :
                inputBitrate(inputBitrate),
                outputBitrate(outputBitrate) {
            ROS_INFO("create converter to convert from %d bit to %d bit", (int) inputBitrate, (int) outputBitrate);
        }


        void Converter::convert_bitrate_to_sox_sample_size(int8_t *framesIn,
                                                           size_t framesAmount,
                                                           sox_sample_t *framesOut) {
            long int i = framesAmount;
            SOX_SAMPLE_LOCALS;
            switch (inputBitrate) {
                case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED: {
                    uint8_t *buf1 = (uint8_t *) framesIn;
                    while (i--) *framesOut++ = SOX_UNSIGNED_8BIT_TO_SAMPLE(*buf1++,);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_8_SIGNED: {
                    int8_t *buf1 = (int8_t *) framesIn;
                    while (i--) *framesOut++ = SOX_SIGNED_8BIT_TO_SAMPLE(*buf1++,);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED: {
                    uint16_t *buf1 = (uint16_t *) framesIn;
                    while (i--) *framesOut++ = SOX_UNSIGNED_16BIT_TO_SAMPLE(*buf1++,);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_16_SIGNED: {
                    int16_t *buf1 = (int16_t *) framesIn;
                    while (i--) *framesOut++ = SOX_SIGNED_16BIT_TO_SAMPLE(*buf1++,);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED: {
                    sox_uint24_t *buf1 = (sox_uint24_t *) framesIn;
                    while (i--) *framesOut++ = SOX_UNSIGNED_24BIT_TO_SAMPLE(*buf1++,);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_24_SIGNED: {
                    sox_int24_t *buf1 = (sox_int24_t *) framesIn;
                    while (i--) *framesOut++ = SOX_SIGNED_24BIT_TO_SAMPLE(*buf1++,);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED: {
                    uint32_t *buf1 = (uint32_t *) framesIn;
                    while (i--) *framesOut++ = SOX_UNSIGNED_32BIT_TO_SAMPLE(*buf1++,);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_32_SIGNED: {
                    int32_t *buf1 = (int32_t *) framesIn;
                    while (i--) *framesOut++ = SOX_SIGNED_32BIT_TO_SAMPLE(*buf1++,);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_FLOAT_32: {
                    float_t *buf1 = (float_t *) framesIn;
                    int clips = 0;
                    while (i--) *framesOut++ = SOX_FLOAT_32BIT_TO_SAMPLE(*buf1++, clips);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_FLOAT_64: {
                    double_t *buf1 = (double_t *) framesIn;
                    int clips = 0;
                    while (i--) *framesOut++ = SOX_FLOAT_64BIT_TO_SAMPLE(*buf1++, clips);
                    break;
                }
                default:
                    std::string ex_text = "bitrate is not supported";
                    throw std::invalid_argument(ex_text);
            }
        }

        void Converter::convert_bitrate_from_sox_sample_size(sox_sample_t *framesIn,
                                                             size_t framesAmount,
                                                             int8_t *framesOut) {
            long int i = framesAmount;
            SOX_SAMPLE_LOCALS;
            int clips = 0;
            switch (outputBitrate) {
                case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED: {
                    uint8_t *buf1 = (uint8_t *) framesOut;
                    while (i--) *buf1++ = SOX_SAMPLE_TO_UNSIGNED_8BIT(*framesIn++, clips);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_8_SIGNED: {
                    int8_t *buf1 = (int8_t *) framesOut;
                    while (i--) *buf1++ = SOX_SAMPLE_TO_SIGNED_8BIT(*framesIn++, clips);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED: {
                    uint16_t *buf1 = (uint16_t *) framesOut;
                    while (i--) *buf1++ = SOX_SAMPLE_TO_UNSIGNED_16BIT(*framesIn++, clips);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_16_SIGNED: {
                    int16_t *buf1 = (int16_t *) framesOut;
                    while (i--) *buf1++ = SOX_SAMPLE_TO_SIGNED_16BIT(*framesIn++, clips);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED: {
                    sox_uint24_t *buf1 = (sox_uint24_t *) framesOut;
                    while (i--) *buf1++ = SOX_SAMPLE_TO_UNSIGNED_24BIT(*framesIn++, clips);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_24_SIGNED: {
                    sox_int24_t *buf1 = (sox_int24_t *) framesOut;
                    while (i--) *buf1++ = SOX_SAMPLE_TO_SIGNED_24BIT(*framesIn++, clips);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED: {
                    uint32_t *buf1 = (uint32_t *) framesOut;
                    while (i--) *buf1++ = SOX_SAMPLE_TO_UNSIGNED_32BIT(*framesIn++, clips);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_32_SIGNED: {
                    int32_t *buf1 = (int32_t *) framesOut;
                    while (i--) *buf1++ = SOX_SAMPLE_TO_SIGNED_32BIT(*framesIn++, clips);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_FLOAT_32: {
                    float_t *buf1 = (float_t *) framesOut;
                    while (i--) *buf1++ = SOX_SAMPLE_TO_FLOAT_32BIT(*framesIn++, clips);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_FLOAT_64: {
                    int64_t *buf1 = (int64_t *) framesOut;
                    while (i--) *buf1++ = SOX_SAMPLE_TO_FLOAT_64BIT(*framesIn++, clips);
                    break;
                }
                default:
                    std::string ex_text = "bitrate is not supported";
                    throw std::invalid_argument(ex_text);
            }
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
