//
// Created by rfeldhans on 02.11.18.
//

//signal handler and file writing
#include <csignal>
#include <fstream>

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
    std::string topicname =  pt.get<std::string>("input_topic");

    // ros initialisation
    ros::init(argc, argv, pt.get<std::string>("node_name"));
    ros::NodeHandle n;

    // file initialization
    std::string file_path = pt.get<std::string>("path_to_file");
    esiaf_ros::utils::autoExpandEnvironmentVariables(file_path);
    std::ofstream fout(file_path, std::ios::out | std::ios::binary);
    if (!fout.is_open()){
        ROS_WARN("File '%s' could not be written, exiting now!", file_path.c_str());
        exit(0);
    }

    //////////////////////////////////////////////////////
    // esiaf start
    //////////////////////////////////////////////////////

    // initialise esiaf
    ROS_INFO("starting esiaf initialisation...");
    esiaf_ros::Esiaf_Handler handler;
    handler.initialize_esiaf(&n, esiaf_ros::NodeDesignation::Other);

    //create format for output topic
    esiaf_ros::EsiafAudioTopicInfo topicInfo;

    esiaf_ros::EsiafAudioFormat allowedFormat;
    allowedFormat.rate = esiaf_ros::utils::cfg_to_esaif_rate(pt.get<int>("sample_rate"));
    allowedFormat.channels = 1;
    allowedFormat.bitrate = esiaf_ros::utils::cfg_to_esiaf_bitrate(pt.get<int>("bitrate"), pt.get<char>("signed_unsigned"), pt.get<char>("int_float"));
    allowedFormat.endian = esiaf_ros::utils::cfg_to_esiaf_endian(pt.get<std::string>("endian"));

    topicInfo.allowedFormat = allowedFormat;
    topicInfo.topic = topicname;

    // notify esiaf about the output topic
    ROS_INFO("adding input topic....");

    // here we add a method to write audio data directly to the file
    simple_esiaf_callback = [&](const std::vector<int8_t> &signal,
                                const esiaf_ros::RecordingTimeStamps &timeStamps){

        fout.write((const char*) signal.data(), signal.size() * sizeof(int8_t));
        fout.flush();
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
        fout.close();
        exit(0);
    };
    signal(SIGINT, signal_handler);

    ROS_INFO("Node ready!");

    ros::spin();


    return (0);
}