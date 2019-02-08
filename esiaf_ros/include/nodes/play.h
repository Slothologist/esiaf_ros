//
// Created by rfeldhans on 24.01.19.
//

#ifndef ESIAF_ROS_PLAY_H
#define ESIAF_ROS_PLAY_H

#include <mutex>
#include <queue>
#include <alsa/asoundlib.h>
#include <esiaf_ros.h>

namespace nodes {

    /**
     * Class to enclose audio playback. Will spawn a dedicated playing thread and offer a method to add audio to play.
     */
    class Player {

    public:
        /**
         * Basic constructor. Takes an alsa pcm handle which must be initialized and a playback device.
         * @param playback_handle The handle which this player will send its audio to.
         */
        Player(snd_pcm_t *playback_handle);

        /**
         * Basic deconstructor.
         */
        ~Player();

        /**
         * Adds audio to an internal audio queue. The audio will be played after all previously added audio has been played.
         * @param audio Pointer to audio data
         * @param size Amount of data points available through the audio ponter
         */
        void add_audio(int16_t *audio, size_t size);

        /**
         * Will stop this audio player and close its pcm handle.
         */
        void stop();

    private:
        /**
         * Method to endlessly play back audio from the audio queue.
         */
        void playThreadMethod();

        /**
         * A queue of pointers where audio, which shall be played, can be found.
         */
        std::queue<int16_t *> playlist;

        /**
         * A queue to keep track of the amount of samples available in each of ponters in playlist.
         */
        std::queue<size_t> playsize;

        /**
         * Mutex lock used to secure the playlist and playsize between the add_audio and playThreadMethod methods.
         */
        std::mutex mutex;

        /**
         * Variable to keep track if this player is running or has stopped.
         */
        bool running;

        /**
         * The alsa pcm handle a . Defines the audio device with which audio will be played back.
         */
        snd_pcm_t *playback_handle;

        /**
         * The dedicated audio playback thread. Runs the appropriately named playThreadMethod.
         */
        std::thread playThread;

    protected:
        //nothing

    };


    /**
     * Takes an esiaf_ros Bitrate and Endian and turns them into the corresponding alsa (asoundlib) equivalent.
     * @param bitrate
     * @param endian
     * @return
     */
    _snd_pcm_format set_up_format_from_bitrate_and_endian(esiaf_ros::Bitrate bitrate, esiaf_ros::Endian endian);

    /**
     * Turns an esiaf_ros Rate into an unsigned int describing the same sample rate.
     * @param sample_rate
     * @return
     */
    unsigned int set_up_sample_rate_from_esiaf(esiaf_ros::Rate sample_rate);

}


#endif //ESIAF_ROS_PLAY_H
