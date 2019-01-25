//
// Created by rfeldhans on 16.01.19.
//

#include "ros/ros.h"
#include <esiaf_ros.h>
#include "esiaf_ros/RecordingTimeStamps.h"
#include <alsa/asoundlib.h>

int main(int argc, char **argv) {

    // some parameters for esiaf
    std::string topicname = "input";

    // ros initialisation
    ros::init(argc, argv, "esiaf_grabber");
    ros::NodeHandle n;

    // alsa initialisation
    int i;
    int err;
    int buffersize = 128;
    int8_t buf[buffersize];
    snd_pcm_t *capture_handle;
    snd_pcm_hw_params_t *hw_params;

    if ((err = snd_pcm_open (&capture_handle, argv[1], SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf (stderr, "cannot open audio device %s (%s)\n",
                 argv[1],
                 snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0) {
        fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n",
                 snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_any (capture_handle, hw_params)) < 0) {
        fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
                 snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_set_access (capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        fprintf (stderr, "cannot set access type (%s)\n",
                 snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_set_format (capture_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
        fprintf (stderr, "cannot set sample format (%s)\n",
                 snd_strerror (err));
        exit (1);
    }

    unsigned int rate = 8000;
    if ((err = snd_pcm_hw_params_set_rate_near (capture_handle, hw_params, &rate, 0)) < 0) {
        fprintf (stderr, "cannot set sample rate (%s)\n",
                 snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_set_channels (capture_handle, hw_params, 1)) < 0) {
        fprintf (stderr, "cannot set channel count (%s)\n",
                 snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params (capture_handle, hw_params)) < 0) {
        fprintf (stderr, "cannot set parameters (%s)\n",
                 snd_strerror (err));
        exit (1);
    }

    snd_pcm_hw_params_free (hw_params);

    if ((err = snd_pcm_prepare (capture_handle)) < 0) {
        fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
                 snd_strerror (err));
        exit (1);
    }

    //////////////////////////////////////////////////////
    // esiaf start
    //////////////////////////////////////////////////////

    // initialise esiaf
    esiaf_ros::initialize_esiaf(&n, esiaf_ros::NodeDesignation::Other);

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
    esiaf_ros::add_output_topic(topicInfo);

    // start esiaf
    esiaf_ros::start_esiaf();

    //////////////////////////////////////////////////////
    // esiaf initialisation finished
    //////////////////////////////////////////////////////

    // grab audio
    while (ros::ok()) {
        ros::Time begin = ros::Time::now();
        if ((err = snd_pcm_readi (capture_handle, buf, buffersize)) != buffersize) {
            fprintf (stderr, "read from audio interface failed (%s)\n",
                     snd_strerror (err));
            exit (1);
        } else{

            // publish audio via esiaf
            ros::Time end = ros::Time::now();
            esiaf_ros::RecordingTimeStamps timeStamps;
            timeStamps.start = begin;
            timeStamps.finish = end;

            std::vector<int8_t> msg_input(buf, buf + buffersize);
            esiaf_ros::publish(topicname, msg_input, timeStamps);
        }
    }

    // close audio device
    snd_pcm_close (capture_handle);

    return(0);
}

