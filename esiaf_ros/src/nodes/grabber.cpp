//
// Created by rfeldhans on 16.01.19.
//

//signal handler and threading
#include <csignal>
#include <thread>

#include <alsa/asoundlib.h>
#include "ros/ros.h"
#include <esiaf_ros.h>
#include "esiaf_ros/RecordingTimeStamps.h"
#include "nodes/utils.h"

// boost config read imports
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

std::function<void(std::vector<int8_t>, const ros::Time &, const ros::Time &)> publish_audio_func;

void publish_audio(std::vector<int8_t> signal, const ros::Time &start_time, const ros::Time &end_time) {
    publish_audio_func(signal, start_time, end_time);
}

int main(int argc, char **argv) {

    std::string config_file = argv[1];

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);

    // some parameters for esiaf
    std::string topicname = pt.get<std::string>("output_topic");

    // ros initialisation
    ros::init(argc, argv, pt.get<std::string>("node_name"));
    ros::NodeHandle n;

    // alsa initialisation
    int i;
    int err;
    int buffersize = 8000;
    snd_pcm_t *capture_handle;
    snd_pcm_hw_params_t *hw_params;

    // setting up hardware parameters
    std::string audio_device = pt.get<std::string>("audio_device");
    esiaf_ros::Rate sample_rate = esiaf_ros::utils::cfg_to_esaif_rate(pt.get<int>("sample_rate"));
    esiaf_ros::Bitrate bitrate = esiaf_ros::utils::cfg_to_esiaf_bitrate(pt.get<int>("bitrate"),
                                                                        pt.get<char>("signed_unsigned"),
                                                                        pt.get<char>("int_float"));
    esiaf_ros::Endian endian = esiaf_ros::utils::cfg_to_esiaf_endian(pt.get<std::string>("endian"));
    auto channels = pt.get <unsigned int> ("channels");

    _snd_pcm_format format = esiaf_ros::utils::set_up_format_from_bitrate_and_endian(bitrate, endian);
    unsigned int sample_rate_ext = esiaf_ros::utils::set_up_sample_rate_from_esiaf(sample_rate);

    //////////////////////////////////////////////////////
    // so much alsa stuff
    //////////////////////////////////////////////////////

    ROS_INFO("preparing audio device...");

    if ((err = snd_pcm_open(&capture_handle, audio_device.c_str(), SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf(stderr, "cannot open audio device %s (%s)\n",
                audio_device.c_str(),
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_malloc(&hw_params)) < 0) {
        fprintf(stderr, "cannot allocate hardware parameter structure (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_any(capture_handle, hw_params)) < 0) {
        fprintf(stderr, "cannot initialize hardware parameter structure (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        fprintf(stderr, "cannot set access type (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_format(capture_handle, hw_params, format)) < 0) {
        fprintf(stderr, "cannot set sample format (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &sample_rate_ext, 0)) < 0) {
        fprintf(stderr, "cannot set sample rate (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_channels(capture_handle, hw_params, channels)) < 0) {
        fprintf(stderr, "cannot set channel count (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params(capture_handle, hw_params)) < 0) {
        fprintf(stderr, "cannot set parameters (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    snd_pcm_hw_params_free(hw_params);

    if ((err = snd_pcm_prepare(capture_handle)) < 0) {
        fprintf(stderr, "cannot prepare audio interface for use (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    snd_pcm_prepare(capture_handle);

    //////////////////////////////////////////////////////
    // esiaf start
    //////////////////////////////////////////////////////

    ROS_INFO("starting esiaf initialisation...");

    // initialise esiaf
    esiaf_ros::esiaf_handle *eh = esiaf_ros::initialize_esiaf(&n, esiaf_ros::NodeDesignation::Other);
    ROS_INFO("creating esiaf output topic...");

    //create format for output topic
    esiaf_ros::EsiafAudioTopicInfo topicInfo;

    esiaf_ros::EsiafAudioFormat allowedFormat;
    allowedFormat.rate = sample_rate;
    allowedFormat.channels = channels;
    allowedFormat.bitrate = bitrate;
    allowedFormat.endian = endian;

    topicInfo.allowedFormat = allowedFormat;
    topicInfo.topic = topicname;

    // notify esiaf about the output topic
    ROS_INFO("adding esiaf output topic...");
    esiaf_ros::add_output_topic(eh, topicInfo);

    // start esiaf
    ROS_INFO("starting esiaf...");
    esiaf_ros::start_esiaf(eh);

    //////////////////////////////////////////////////////
    // esiaf initialisation finished
    //////////////////////////////////////////////////////



    //  we want the audio grabber to do just that, so here we create a function to publish our signal.
    publish_audio_func = [&](std::vector<int8_t> signal, const ros::Time &start_time, const ros::Time &end_time) {

        // create timestamps
        esiaf_ros::RecordingTimeStamps timeStamps;
        timeStamps.start = start_time;
        timeStamps.finish = end_time;

        // publish everything
        esiaf_ros::publish(eh, topicname, signal, timeStamps);
    };

    // initialize timestamps for later publishing
    ros::Time begin = ros::Time::now(), end;

    // here we create a dedicated thread to grab audio
    std::thread audioGrabberThread([&] {
        while (true) {
            int16_t buffer[buffersize];
            if ((err = snd_pcm_readi(capture_handle, buffer, buffersize)) != buffersize) {
                ROS_ERROR("read from audio interface failed (%s)",
                          snd_strerror(err));
            } else {
                end = ros::Time::now();

                // pack buffer into std::vector
                int8_t *buf8 = (int8_t *) buffer;
                std::vector<int8_t> signal(buf8, buf8 + (buffersize * (sizeof(int16_t)/sizeof(int8_t))));

                // create a new thread to publish the captured audio and let it do its thing (detach)
                std::thread publishThread(publish_audio, signal, begin, end);
                publishThread.detach();

                // dont forget to switch begin to end timestamp
                begin = end;
            }
        }
    });

    ROS_INFO("Node ready!");
    ros::spin();

    // close audio device
    snd_pcm_close(capture_handle);
    esiaf_ros::quit_esiaf();

    return (0);
}

