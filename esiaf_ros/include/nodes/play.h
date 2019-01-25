//
// Created by rfeldhans on 24.01.19.
//

#ifndef ESIAF_ROS_PLAY_H
#define ESIAF_ROS_PLAY_H

void esiaf_callback(std::vector<int8_t> signal, esiaf_ros::RecordingTimeStamps timeStamps, snd_pcm_t* playback_handle);

#endif //ESIAF_ROS_PLAY_H
