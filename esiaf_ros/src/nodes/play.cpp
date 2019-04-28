//
// Created by rfeldhans on 02.11.18.
//

//signal handler and threading
#include <csignal>
#include <thread>
#include <chrono>

// ros and esiaf imports
#include "ros/ros.h"
#include "esiaf_ros/RecordingTimeStamps.h"
#include "nodes/play.h"
#include "nodes/utils.h"

// boost config read imports
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

namespace nodes {

    Player::Player(snd_pcm_t *playback_handle) :
            running(true),
            skippedFirstFrame(0),
            playback_handle(playback_handle) {
        playThread = std::thread(&Player::playThreadMethod, this);
    }

    Player::~Player() {
        if (running)
            stop();
    }

    void Player::add_audio(int16_t *audio, size_t size) {
        mutex.lock();
        if(skippedFirstFrame == 0 || skippedFirstFrame == 1){
            skippedFirstFrame++;
        }

        playlist.push(audio);
        playsize.push(size);

        mutex.unlock();
    }

    void Player::stop() {
        running = false;
        playThread.join();
        snd_pcm_close(playback_handle); // takes forever, todo: investigate
    }

    void Player::playThreadMethod() {

        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            mutex.lock();
            if(skippedFirstFrame < 2){
                mutex.unlock();
                continue;
            }


            if (!playlist.empty()) {

                int16_t *buf = playlist.front();
                size_t size = playsize.front();
                playlist.pop();
                playsize.pop();

                mutex.unlock();

                snd_pcm_sframes_t frames = snd_pcm_writei(playback_handle, buf, size);

                // Check for errors
                if (frames < 0)
                    frames = snd_pcm_recover(playback_handle, frames, 0);
                if (frames < 0) {
                    fprintf(stderr, "ERROR: Failed writing audio with snd_pcm_writei(): %li\n", frames);
                    exit(EXIT_FAILURE);
                }
                if (frames > 0 && frames < size)
                    printf("Short write (expected %li, wrote %li)\n", size, frames);


            } else {
                mutex.unlock();
            }
        }
    }

}


std::function<void(int)> shutdown_handler;

void signal_handler(int signal) { shutdown_handler(signal); }

boost::function<void(const std::vector<int8_t> &, const esiaf_ros::RecordingTimeStamps &)> simple_esiaf_callback;

void esiaf_handler(const std::vector<int8_t> &signal,
                   const esiaf_ros::RecordingTimeStamps &timeStamps) { simple_esiaf_callback(signal, timeStamps); };


int main(int argc, char **argv) {

    std::string config_file = argv[1];

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);

    // some parameters for esiaf
    std::string topicname = pt.get<std::string>("input_topic");

    // ros initialisation
    ros::init(argc, argv, pt.get<std::string>("node_name"));
    ros::NodeHandle n;

    // alsa initialisation
    int i;
    int err;
    snd_pcm_t *playback_handle;
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

    ROS_INFO("preparing audio device...");
    if ((err = snd_pcm_open(&playback_handle, audio_device.c_str(), SND_PCM_STREAM_PLAYBACK,
                            0)) < 0) {
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

    if ((err = snd_pcm_hw_params_set_format(playback_handle, hw_params, format)) < 0) {
        fprintf(stderr, "cannot set sample format (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_rate_near(playback_handle, hw_params, &sample_rate_ext, 0)) < 0) {
        fprintf(stderr, "cannot set sample rate (%s)\n",
                snd_strerror(err));
        exit(1);
    }

    if ((err = snd_pcm_hw_params_set_channels(playback_handle, hw_params, channels)) < 0) {
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

    snd_pcm_prepare(playback_handle);

    // add an audio player
    nodes::Player player(playback_handle);

    //////////////////////////////////////////////////////
    // esiaf start
    //////////////////////////////////////////////////////

    // initialise esiaf
    ROS_INFO("starting esiaf initialisation...");
    esiaf_ros::Esiaf_Handler handler(&n, esiaf_ros::NodeDesignation::Other);

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
    ROS_INFO("adding input topic....");

    // here we add a method to transfer incoming audio to the audio player
    simple_esiaf_callback = [&](const std::vector<int8_t> &signal,
                                const esiaf_ros::RecordingTimeStamps &timeStamps) {

        const int8_t* buf8 = signal.data();
        int16_t* buf16 = (int16_t*) buf8;

        player.add_audio(buf16, signal.size() / (sizeof(int16_t)/sizeof(int8_t)) );

    };

    handler.add_input_topic(topicInfo, esiaf_handler);

    // start esiaf
    ROS_INFO("starting esiaf...");

    handler.start_esiaf();

    //////////////////////////////////////////////////////
    // esiaf initialisation finished
    //////////////////////////////////////////////////////

    ROS_INFO("adding signal handler...");
    shutdown_handler = [&](int signal) {
        ROS_INFO("shutting down...");
        player.stop();
        exit(signal);
    };
    signal(SIGINT, signal_handler);

    ROS_INFO("Node ready!");

    ros::spin();

    snd_pcm_close(playback_handle);

    return (0);
}
