//
// Created by rfeldhans on 17.01.19.
//


#include "../include/resample.h"
#include <stdexcept>

namespace esiaf_ros {
    namespace resampling {

        void print_a_few_samples(sox_sample_t* pointy){
            std::string samples = "Samples: ";
            for(int i = 0; i < 100; i++){
                samples.append(" %d,",pointy[i]);
            }

            ROS_INFO(samples.c_str());
        }


        Resampler::Resampler(esiaf_ros::EsiafAudioFormat inputFormat, esiaf_ros::EsiafAudioFormat outputFormat) :
                inputFormat(inputFormat),
                outputFormat(outputFormat),
                converter_to_soxr_sample_size(inputFormat.bitrate, soxr_bitrate),
                converter_from_soxr_sample_size(soxr_bitrate, outputFormat.bitrate)
        {
            /*ROS_INFO("created resampler, which will resample from \n%d hz, %d bit, %d endian to\n%d hz, %d bit, %d endian",
                     inputFormat.rate, inputFormat.bitrate, inputFormat.endian,
                     outputFormat.rate, outputFormat.bitrate, outputFormat.endian);*/
            ROS_INFO("create resampler which will resample from %dHz to %dHz", (int) this->inputFormat.rate,
                     (int) this->outputFormat.rate);
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

            if(inputFormat.rate != outputFormat.rate){
                soxr = soxr_create(
                        irate, orate, inputFormat.channels,             /* Input rate, output rate, # of channels. */
                        &error,                         /* To report any error during creation. */
                        NULL,  // use defaults
                        NULL,  // use defaults
                        NULL); // use defaults
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
                    (sizeof(int8_t) * inputSizeInInt8) / (esiaf_ros::converting::byterate_from_esiaf(inputFormat.bitrate));

            // second converting from input to soxr bitrate
            sox_sample_t *resampleInputBuffer = (sox_sample_t *) malloc(
                    inputFramesAmount * sizeof *resampleInputBuffer);

            // third output
            auto outputFramesAmount = (size_t) ceil(inputFramesAmount * (orate / irate));
            sox_sample_t *resampleOutputBuffer = (sox_sample_t *) malloc(
                    outputFramesAmount * sizeof *resampleOutputBuffer);

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
                converter_to_soxr_sample_size.convert_bitrate(inBuf, inputFramesAmount, (int8_t *) resampleInputBuffer);
            } else {
                //free(resampleInputBuffer); // TODO performance improvements
                resampleInputBuffer = (sox_sample_t*) inBuf;
            }

            ROS_INFO("post bitrate converting 1, pre soxr resampling");

            if (irate != orate){
                soxr_process(soxr, resampleInputBuffer, inputFramesAmount, NULL, resampleOutputBuffer,
                             outputFramesAmount, &written);
            } else {
                //free(resampleOutputBuffer); // TODO performance improvements
                resampleOutputBuffer = resampleInputBuffer;
                written = inputFramesAmount;
            }

            ROS_INFO("resample complete, pre buffer free-ing");

            //free(resampleInputBuffer);
            //resampleInputBuffer = NULL;


            ROS_INFO("post buffer free-ing, pre output buffer allocating");

            // now malloc the outBuffer
            outBuf = (int8_t *) malloc(written * sizeof(sox_sample_t));

            ROS_INFO("post output buffer allocating, pre bitrate converting 2");

            ROS_INFO("written in frames: %ld", written);

            // prepare output for resampling
            // change bitrate from soxr requested to required
            if (outputFormat.bitrate != soxr_bitrate) {
                ROS_INFO("second convert");
                converter_from_soxr_sample_size.convert_bitrate((int8_t*) resampleOutputBuffer, written, outBuf);
            } else {
                //free(outBuf); // TODO performance improvements
                outBuf = (int8_t*) resampleInputBuffer;
            }

            ROS_INFO("post bitrate converting 2, pre resampleOutputBuffer freeing");

            //free(resampleOutputBuffer);
            //resampleOutputBuffer = NULL;
            ROS_INFO("post resampleOutputBuffer freeing, pre vector creation");

            // change input signal vector to resampled result
            std::vector<int8_t> newsignal(outBuf, outBuf + written * sizeof(sox_sample_t));

            ROS_INFO("output size in int8_t: %ld", newsignal.size());

            //free(outBuf);
            //outBuf = NULL;


            ROS_INFO("post vector creation");
            return newsignal;
        }

    }// namespace
}// namespace