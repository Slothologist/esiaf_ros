//
// Created by rfeldhans on 16.08.19.
//

//signal handler and file writing
#include <csignal>

// ros and esiaf imports
#include "ros/ros.h"
#include <esiaf_ros.h>
#include "esiaf_ros/RecordingTimeStamps.h"
#include "nodes/utils.h"

// boost config read imports
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }

boost::function<void(const std::vector<int8_t> &, const esiaf_ros::RecordingTimeStamps &)> simple_esiaf_callback;
void esiaf_handler(const std::vector<int8_t> &signal, const esiaf_ros::RecordingTimeStamps & timeStamps){ simple_esiaf_callback(signal, timeStamps); };


int main(int argc, char **argv) {

    // setting up config file and property tree
    std::string config_file = argv[1];
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);

    // some parameters for esiaf
    std::string topicname_input =  pt.get<std::string>("input_topic");
    std::string topicname_output = pt.get<std::string>("output_topic_prefix");
    int channels = pt.get<int>("channels");

    // ros initialisation
    ros::init(argc, argv, "esiaf_splitter_cpp");
    ros::NodeHandle n;

    //////////////////////////////////////////////////////
    // esiaf start
    //////////////////////////////////////////////////////

    // initialise esiaf
    ROS_INFO("starting esiaf initialisation...");
    esiaf_ros::Esiaf_Handler handler(&n, esiaf_ros::NodeDesignation::Other);

    //create format for output topic
    esiaf_ros::EsiafAudioTopicInfo topicInfo;

    esiaf_ros::EsiafAudioFormat allowedFormat;
    allowedFormat.rate = esiaf_ros::utils::cfg_to_esaif_rate(pt.get<int>("sample_rate"));
    allowedFormat.channels = 1;
    allowedFormat.bitrate = esiaf_ros::utils::cfg_to_esiaf_bitrate(pt.get<int>("bitrate"), pt.get<char>("signed_unsigned"), pt.get<char>("int_float"));
    allowedFormat.endian = esiaf_ros::utils::cfg_to_esiaf_endian(pt.get<std::string>("endian"));

    topicInfo.allowedFormat = allowedFormat;

    for(int i = 0; i < channels; i++){
        topicInfo.topic = topicname_output + std::to_string(i);
        handler.add_output_topic(topicInfo);
    }

    topicInfo.topic = topicname_input;
    topicInfo.allowedFormat.channels = channels;

    // notify esiaf about the output topic
    ROS_INFO("adding input topic....");

    // here we add a method to write audio data directly to the file
    simple_esiaf_callback = [&](const std::vector<int8_t> &signal,
                                const esiaf_ros::RecordingTimeStamps &timeStamps){
        const int8_t* buf8 = signal.data();
        int16_t* buf16 = (int16_t*) buf8;
        std::vector<int16_t> vec16(buf16, buf16 + signal.size()/2);
        std::vector<int16_t> vec16_0;
        std::vector<int16_t> vec16_1;
        std::vector<int16_t> vec16_2;
        std::vector<int16_t> vec16_3;
        for(int i = 0; i < vec16.size()/4; i++){
            vec16_0.push_back(vec16[4*i+0]);
            vec16_1.push_back(vec16[4*i+1]);
            vec16_2.push_back(vec16[4*i+2]);
            vec16_3.push_back(vec16[4*i+3]);
        }
        const int8_t* buf8_0 = (int8_t*) vec16_0.data();
        const int8_t* buf8_1 = (int8_t*) vec16_1.data();
        const int8_t* buf8_2 = (int8_t*) vec16_2.data();
        const int8_t* buf8_3 = (int8_t*) vec16_3.data();

        std::vector<int8_t> vec8_0(buf8_0, buf8_0 + signal.size()/4);
        std::vector<int8_t> vec8_1(buf8_1, buf8_1 + signal.size()/4);
        std::vector<int8_t> vec8_2(buf8_2, buf8_2 + signal.size()/4);
        std::vector<int8_t> vec8_3(buf8_3, buf8_3 + signal.size()/4);

        handler.publish(topicname_output + "0", vec8_0, timeStamps);
        handler.publish(topicname_output + "1", vec8_1, timeStamps);
        handler.publish(topicname_output + "2", vec8_2, timeStamps);
        handler.publish(topicname_output + "3", vec8_3, timeStamps);
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
        exit(0);
    };
    signal(SIGINT, signal_handler);

    ROS_INFO("Node ready!");

    ros::spin();


    return (0);
}
