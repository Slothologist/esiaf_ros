//
// Created by rfeldhans on 17.01.19.
//


#include "../include/resample.h"
#include <inttypes.h>
#include <stdexcept>

namespace esiaf_ros {
    namespace resampling {

        void print_a_few_samples(sox_sample_t *pointy) {
            std::string samples = "Samples: ";
            for (int i = 0; i < 100; i++) {
                samples.append(" %d,", (unsigned long) pointy[i]);
            }

            ROS_INFO(samples.c_str());
        }


        Resampler::Resampler(esiaf_ros::EsiafAudioFormat inputFormat, esiaf_ros::EsiafAudioFormat outputFormat) :
                inputFormat(inputFormat),
                outputFormat(outputFormat),
                converter(inputFormat.bitrate, outputFormat.bitrate) {
            /*ROS_INFO("created resampler, which will resample from \n%d hz, %d bit, %d endian to\n%d hz, %d bit, %d endian",
                     inputFormat.rate, inputFormat.bitrate, inputFormat.endian,
                     outputFormat.rate, outputFormat.bitrate, outputFormat.endian);*/
            ROS_INFO("create resampler which will resample from %dHz to %dHz", (int) this->inputFormat.rate,
                     (int) this->outputFormat.rate);
            if (inputFormat.channels != outputFormat.channels) {
                std::string ex_text = "Different channel count is not supported for resampling yet";
                throw std::invalid_argument(ex_text);
            }
            irate = sox_rate_from_esiaf(inputFormat.rate);
            orate = sox_rate_from_esiaf(outputFormat.rate);


            if (inputFormat.rate != outputFormat.rate) {
                setup_resampler();
                ROS_INFO("resampler created which will resample from %dHz to %dHz", (int) inputFormat.rate,
                         (int) outputFormat.rate);
            } else {
                ROS_INFO("No resampler needed for same samplerate");
            }
        }

        std::vector<int8_t> Resampler::resample(const std::vector<int8_t> &signal) {
            ROS_INFO("Resampling step taking place!");
            // calculate sizes of various stages
            // first input
            size_t inputSizeInInt8 = signal.size();
            int8_t *inBuf = (int8_t *) signal.data();
            size_t inputFramesAmount =
                    (sizeof(int8_t) * inputSizeInInt8) /
                    esiaf_ros::converting::byterate_from_esiaf(inputFormat.bitrate);

            // second converting from input to soxr bitrate
            sox_sample_t *resampleInputBuffer = new sox_sample_t[inputFramesAmount];

            // third output
            auto outputFramesAmount = (size_t) ceil(inputFramesAmount * (orate / irate));
            sox_sample_t *resampleOutputBuffer = new sox_sample_t[outputFramesAmount];

            // fourth converting from soxr to output bitrate
            size_t written = 1;
            int8_t *outBuf; // neeeds to be malloc'd after written is actually determined


            ROS_INFO("input size in int8: %ld", inputSizeInInt8);
            ROS_INFO("input size in frames: %ld", inputFramesAmount);
            ROS_INFO("input bitrate: %d", esiaf_ros::converting::byterate_from_esiaf(inputFormat.bitrate));
            ROS_INFO("sizeof(int8_t): %ld", sizeof(int8_t));
            ROS_INFO("irate: %f; orate: %f", irate, orate);
            ROS_INFO("output size in frames: %ld", outputFramesAmount);


            ROS_INFO("Most buffers allocated, pre bitrate converting 1");

            // prepare input for resampling
            // change bitrate to soxr requested
            if (inputFormat.bitrate != soxr_bitrate) {
                ROS_INFO("first convert");
                converter.convert_bitrate_to_sox_sample_size(inBuf, inputFramesAmount, resampleInputBuffer);
            } else {
                resampleInputBuffer = (sox_sample_t *) inBuf;
            }

            ROS_INFO("post bitrate converting 1, pre soxr resampling");

            if (irate != orate) {
                soxr_process(soxr,
                             resampleInputBuffer,
                             inputFramesAmount,
                             NULL,
                             resampleOutputBuffer,
                             outputFramesAmount,
                             &written);
            } else {
                resampleOutputBuffer = resampleInputBuffer;
                written = inputFramesAmount;
            }

            ROS_INFO("resample complete, pre output buffer allocating");

            // now malloc the outBuffer
            size_t outBufSize = written * esiaf_ros::converting::byterate_from_esiaf(outputFormat.bitrate);
            outBuf = new int8_t[outBufSize];

            ROS_INFO("post output buffer allocating, pre bitrate converting 2");

            ROS_INFO("written in frames: %ld", written);

            // prepare output for resampling
            // change bitrate from soxr requested to required
            if (outputFormat.bitrate != soxr_bitrate) {
                ROS_INFO("second convert");
                converter.convert_bitrate_from_sox_sample_size(resampleOutputBuffer, written, outBuf);
            } else {
                outBuf = (int8_t *) resampleInputBuffer;
            }

            ROS_INFO("post bitrate converting 2, pre vector creation");

            // change input signal vector to resampled result
            std::vector<int8_t> newsignal(outBuf, outBuf + outBufSize);

            ROS_INFO("output size in int8_t: %ld", newsignal.size());

            /**
             * In theory we would need these three deletes, to free the dynamically allocated memory inside this method.
             * However, because of some reason, the resampling
             * delete[] resampleInputBuffer;
             * delete[] resampleOutputBuffer;
             * delete[] outBuf;
             */

            ROS_INFO("post vector creation");
            return newsignal;
        }

        double sox_rate_from_esiaf(esiaf_ros::Rate rate) {
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

        void Resampler::setup_resampler() {
            soxr_error_t error;

            soxr_io_spec_t io_spec;// use defaults
            io_spec.itype=SOXR_INT32_I;
            io_spec.otype= SOXR_INT32_I;
            io_spec.scale = 1;
            io_spec.e = 0;
            io_spec.flags = 0;

            soxr_quality_spec_t qual_spec;// use defaults
            qual_spec.precision = 20;
            qual_spec.phase_response =50;
            qual_spec.passband_end = 0.913;
            qual_spec.stopband_begin = 1;
            qual_spec.e = 0;
            qual_spec.flags = 0;

            soxr_runtime_spec_t runtime_spec;// use defaults
            runtime_spec.log2_min_dft_size = 10;
            runtime_spec.log2_large_dft_size = 17;
            runtime_spec.coef_size_kbytes = 400;
            runtime_spec.num_threads = 1;
            runtime_spec.flags = 0;
            runtime_spec.e = 0;

            soxr = soxr_create(
                    irate, orate, inputFormat.channels,             /* Input rate, output rate, # of channels. */
                    &error,                         /* To report any error during creation. */
                    &io_spec,
                    &qual_spec,
                    &runtime_spec);
        }

    }// namespace
}// namespace