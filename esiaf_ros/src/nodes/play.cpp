//
// Created by rfeldhans on 02.11.18.
//

#include <csignal>
#include <mutex>

#include "ros/ros.h"
#include <esiaf_ros.h>
#include "esiaf_ros/RecordingTimeStamps.h"
#include <alsa/asoundlib.h>
#include "../../include/nodes/play.h"


std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }

boost::function<void(const std::vector<int8_t> &, const esiaf_ros::RecordingTimeStamps &)> simple_esiaf_callback;
void esiaf_handler(const std::vector<int8_t> &signal, const esiaf_ros::RecordingTimeStamps & timeStamps){ simple_esiaf_callback(signal, timeStamps); };





int main(int argc, char **argv) {
    ROS_INFO("MAIN CALLED");

    // some parameters for esiaf
    std::string topicname = "input";

    // ros initialisation
    ros::init(argc, argv, "esiaf_grabber");
    ros::NodeHandle n;

    int i;
    int err;
    snd_pcm_t *playback_handle;
    snd_pcm_hw_params_t *hw_params;

    ROS_INFO("preparing audio device...");
    if ((err = snd_pcm_open(&playback_handle, argv[1], SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
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

    if ((err = snd_pcm_hw_params_any(playback_handle, hw_params)) < 0) {
        fprintf(stderr, "cannot initialize hardware parameter structure (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_access(playback_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        fprintf(stderr, "cannot set access type (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_format(playback_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
        fprintf(stderr, "cannot set sample format (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    unsigned int rate = 8000;
    if ((err = snd_pcm_hw_params_set_rate_near(playback_handle, hw_params, &rate, 0)) < 0) {
        fprintf(stderr, "cannot set sample rate (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_channels(playback_handle, hw_params, 1)) < 0) {
        fprintf(stderr, "cannot set channel count (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params(playback_handle, hw_params)) < 0) {
        fprintf(stderr, "cannot set parameters (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    snd_pcm_hw_params_free(hw_params);

    if ((err = snd_pcm_prepare(playback_handle)) < 0) {
        fprintf(stderr, "cannot prepare audio interface for use (%s)\n",
                snd_strerror(err));
        exit(1);
    }



    //////////////////////////////////////////////////////
    // esiaf start
    //////////////////////////////////////////////////////

    // initialise esiaf
    ROS_INFO("starting esiaf initialisation...");
    esiaf_ros::esiaf_handle *eh = esiaf_ros::initialize_esiaf(&n, esiaf_ros::NodeDesignation::Other);

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
    ROS_INFO("adding input topic....");
    std::mutex mtx;


    simple_esiaf_callback = [&](const std::vector<int8_t> &signal,
                                const esiaf_ros::RecordingTimeStamps &timeStamps){
        ROS_INFO("Callback node start");
        int err;

        const int8_t *buf8 = signal.data();

        int16_t *buf16[signal.size()/2];
        mempcpy(buf16, buf8, sizeof(buf16));

        // Write data and play sound (blocking)
        mtx.lock();
        snd_pcm_sframes_t frames = snd_pcm_writei(playback_handle, buf16, signal.size()/2);

        ROS_INFO("frames %d", frames);

        // Check for errors
        if (frames < 0)
            //ROS_INFO("recover");
            frames = snd_pcm_recover(playback_handle, frames, 0);
        if (frames < 0) {
            fprintf(stderr, "ERROR: Failed writing audio with snd_pcm_writei(): %li\n", frames);
            exit(EXIT_FAILURE);
        }
        if (frames > 0 && frames < signal.size()/2)
            printf("Short write (expected %d, wrote %li)\n", signal.size()/2, frames);
        mtx.unlock();
        ROS_INFO("Callback node end");
    };

    esiaf_ros::add_input_topic(eh, topicInfo, esiaf_handler);

    // start esiaf
    ROS_INFO("starting esiaf...");

    esiaf_ros::start_esiaf(eh);

    //////////////////////////////////////////////////////
    // esiaf initialisation finished
    //////////////////////////////////////////////////////

    ROS_INFO("adding signal handler...");
    shutdown_handler = [&](int signal) {
        ROS_INFO("shutting down...");
        snd_pcm_close(playback_handle);
        exit(signal);
    };
    signal(SIGINT, signal_handler);


    snd_pcm_prepare(playback_handle);
    ROS_INFO("Node ready!");

    ros::spin();

    //snd_pcm_close(playback_handle);

    return (0);
}
