//
// Created by rfeldhans on 16.01.19.
//

#include <thread>
#include "ros/ros.h"
#include <esiaf_ros.h>
#include "esiaf_ros/RecordingTimeStamps.h"
#include <alsa/asoundlib.h>

std::function<void(std::vector<int8_t>, const ros::Time &, const ros::Time &)> publish_audio_func;

void publish_audio(std::vector<int8_t> signal, const ros::Time &start_time, const ros::Time &end_time) {
    publish_audio_func(signal, start_time, end_time);
}

int main(int argc, char **argv) {

    // some parameters for esiaf
    std::string topicname = "input";

    // ros initialisation
    ros::init(argc, argv, "esiaf_grabber");
    ros::NodeHandle n;

    // alsa initialisation
    int i;
    int err;
    int buffersize = 8000;
    snd_pcm_t *capture_handle;
    snd_pcm_hw_params_t *hw_params;

    //////////////////////////////////////////////////////
    // so much alsa stuff
    //////////////////////////////////////////////////////

    ROS_INFO("preparing audio device...");

    if ((err = snd_pcm_open(&capture_handle, argv[1], SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf(stderr, "cannot open audio device %s (%s)\n",
                argv[1],
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

    if ((err = snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
        fprintf(stderr, "cannot set sample format (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    unsigned int rate = 8000;
    if ((err = snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &rate, 0)) < 0) {
        fprintf(stderr, "cannot set sample rate (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_channels(capture_handle, hw_params, 1)) < 0) {
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
    allowedFormat.rate = esiaf_ros::Rate::RATE_8000;
    allowedFormat.channels = 1;
    allowedFormat.bitrate = esiaf_ros::Bitrate::BIT_8;
    allowedFormat.endian = esiaf_ros::Endian(1);

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
                std::vector<int8_t> signal(buf8, buf8 + 2 * buffersize);

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

    return (0);
}

