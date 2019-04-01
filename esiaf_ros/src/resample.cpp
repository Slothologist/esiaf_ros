//
// Created by rfeldhans on 17.01.19.
//


#include "../include/resample.h"
#include <sox.h>
#include <stdexcept>
#include <assert.h>

namespace esiaf_ros {
    namespace resampling {


        Resampler::Resampler(esiaf_ros::EsiafAudioFormat inputFormat, esiaf_ros::EsiafAudioFormat outputFormat) :
                inputFormat(inputFormat),
                outputFormat(outputFormat) {
            ROS_INFO(
                    "created resampler, which will resample from \n%d hz, %d bit, %d endian, %d channel to\n%d hz, %d bit, %d endian, %d channel",
                    inputFormat.rate, inputFormat.bitrate, inputFormat.endian, inputFormat.channels,
                    outputFormat.rate, outputFormat.bitrate, outputFormat.endian, outputFormat.channels);

            irate = sox_rate_from_esiaf(inputFormat.rate);
            orate = sox_rate_from_esiaf(outputFormat.rate);
        }

        std::vector<int8_t> Resampler::resample(const std::vector<int8_t> &signal) {

            int inputBitrate = bitrate_from_esiaf(inputFormat.bitrate);
            int outputBitrate = bitrate_from_esiaf(outputFormat.bitrate);

            // figure out buffer and buffersizes
            internal_input_buffer_size = signal.size() * sizeof(int8_t);
            internal_input_buffer = new int8_t[signal.size()];
            std::copy(signal.data(), signal.data() + signal.size() * sizeof(int8_t), internal_input_buffer);

            double output_scaling_factor = (orate / irate) * ((double)outputBitrate / (double)inputBitrate);
            internal_output_buffer_size =  (size_t)(((output_scaling_factor * signal.size())) + 0.5);
            internal_output_buffer = new int8_t[internal_output_buffer_size];

            setup_sox_effect_chain();

            sox_flow_effects(chain, NULL, NULL);
            // change input signal vector to resampled result
            std::vector<int8_t> newsignal(internal_output_buffer, internal_output_buffer + internal_output_buffer_size);

            // tidy up effect chain etc
            sox_delete_effects_chain(chain);
            sox_close(out);
            sox_close(in);
            free(internal_input_buffer);
            free(internal_output_buffer);

            return newsignal;
        }

        void Resampler::setup_sox_effect_chain() {

            //ROS_INFO("resampling!");
            setup_sox_formats_from_esiaf();
            sox_effect_t *e;
            char *args[10];
            chain = sox_create_effects_chain(&in->encoding, &out->encoding);
            sox_signalinfo_t interm_signal = in->signal; /* NB: deep copy */

            //ROS_INFO("before effect adding");

            e = sox_create_effect(sox_find_effect("input"));
            args[0] = (char *) in, assert(sox_effect_options(e, 1, args) == SOX_SUCCESS);
            assert(sox_add_effect(chain, e, &interm_signal, &in->signal) == SOX_SUCCESS);
            free(e);

            //ROS_INFO("after input adding");


            if (in->signal.rate != out->signal.rate) {
                e = sox_create_effect(sox_find_effect("rate"));
                assert(sox_effect_options(e, 0, NULL) == SOX_SUCCESS);
                assert(sox_add_effect(chain, e, &interm_signal, &out->signal) == SOX_SUCCESS);
                free(e);
                //ROS_INFO("after rate effect adding");
            }

            if (in->signal.channels != out->signal.channels) {
                e = sox_create_effect(sox_find_effect("channels"));
                assert(sox_effect_options(e, 0, NULL) == SOX_SUCCESS);
                assert(sox_add_effect(chain, e, &interm_signal, &out->signal) == SOX_SUCCESS);
                free(e);
                //ROS_INFO("after channel effect adding");
            }

            e = sox_create_effect(sox_find_effect("output"));
            args[0] = (char *) out, assert(sox_effect_options(e, 1, args) == SOX_SUCCESS);
            assert(sox_add_effect(chain, e, &interm_signal, &out->signal) == SOX_SUCCESS);
            free(e);

            //ROS_INFO("after output adding");
        }


        void Resampler::setup_sox_formats_from_esiaf() {
            sox_signalinfo_t input_format = sox_signalinfo_from_esiaf(inputFormat);
            sox_signalinfo_t output_format = sox_signalinfo_from_esiaf(outputFormat);
            sox_encodinginfo_t input_encoding = sox_encodinginfo_from_esiaf(inputFormat);
            sox_encodinginfo_t output_encoding = sox_encodinginfo_from_esiaf(outputFormat);
            assert(in = sox_open_mem_read(internal_input_buffer, internal_input_buffer_size, &input_format,
                                          &input_encoding,
                                          "raw"));
            assert(out = sox_open_mem_write(internal_output_buffer, internal_output_buffer_size, &output_format,
                                            &output_encoding,
                                            "raw", NULL));

        }

        sox_encodinginfo_t Resampler::sox_encodinginfo_from_esiaf(esiaf_ros::EsiafAudioFormat format) {
            sox_encodinginfo_t encodinginfo;
            encodinginfo.bits_per_sample = (unsigned int) bitrate_from_esiaf(format.bitrate);
            switch (format.bitrate) {
                case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED:
                case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED:
                    encodinginfo.encoding = SOX_ENCODING_UNSIGNED;
                    break;
                case esiaf_ros::Bitrate::BIT_INT_8_SIGNED:
                case esiaf_ros::Bitrate::BIT_INT_16_SIGNED:
                case esiaf_ros::Bitrate::BIT_INT_24_SIGNED:
                case esiaf_ros::Bitrate::BIT_INT_32_SIGNED:
                    encodinginfo.encoding = SOX_ENCODING_SIGN2;
                    break;
                case esiaf_ros::Bitrate::BIT_FLOAT_32:
                case esiaf_ros::Bitrate::BIT_FLOAT_64:
                    encodinginfo.encoding = SOX_ENCODING_FLOAT;
                    break;
                default:
                    char buff[100];
                    snprintf(buff, sizeof(buff), "bitrate is not supported (%d)", (int) format.bitrate);
                    std::string ex_text = buff;
                    throw std::invalid_argument(ex_text);
            }
            encodinginfo.compression = 1.0;
            encodinginfo.opposite_endian = sox_false;
            encodinginfo.reverse_bits = sox_option_default;
            encodinginfo.reverse_bytes = sox_option_default;
            encodinginfo.reverse_nibbles = sox_option_default;
            return encodinginfo;
        }

        sox_signalinfo_t Resampler::sox_signalinfo_from_esiaf(esiaf_ros::EsiafAudioFormat format) {
            sox_signalinfo_t signalinfo;
            signalinfo.channels = format.channels > 0 ? (unsigned int) format.channels : 1;
            signalinfo.precision = (unsigned int) bitrate_from_esiaf(format.bitrate);
            signalinfo.length = SOX_UNSPEC;
            signalinfo.rate = sox_rate_from_esiaf(format.rate);
            signalinfo.mult = NULL;
            return signalinfo;
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
                    char buff[100];
                    snprintf(buff, sizeof(buff), "bitrate is not supported (%d)", (int) bitrate);
                    std::string ex_text = buff;
                    throw std::invalid_argument(ex_text);
            }
        }

        double Resampler::sox_rate_from_esiaf(esiaf_ros::Rate rate) {
            switch (rate) {
                case esiaf_ros::Rate::RATE_8000:
                    return 8000.;
                case esiaf_ros::Rate::RATE_16000:
                    return 16000.;
                case esiaf_ros::Rate::RATE_32000:
                    return 32000.;
                case esiaf_ros::Rate::RATE_44100:
                    return 44100.;
                case esiaf_ros::Rate::RATE_48000:
                    return 48000.;
                case esiaf_ros::Rate::RATE_96000:
                    return 96000.;
                default:
                    std::string ex_text = "sample rate is not supported";
                    throw std::invalid_argument(ex_text);
            }
        }


    }// namespace
}// namespace