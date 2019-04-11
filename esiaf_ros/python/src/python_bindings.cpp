//
// Created by rfeldhans on 08.04.19.
//


#include <string>
#include <boost/python.hpp>
#include "ros/ros.h"
#include "../../include/esiaf_ros.h"

namespace esiaf_ros {

    /**
     * This class is necessary, because rospy.init() and ros::init() are distince from one another and rospy has no method
     * of obtaining the ros::NodeHandle necessary to initialize the EsiafHandler.
     */
    class PyEsiaf_Handler : Esiaf_Handler {
    public:
        void initialize_wrapper(std::string nodeName,
                                NodeDesignation nodeDesignation) {

            int argc = 1;
            // we init ros with the name of the node, not the argv[0]. Lets see how well we are off with this.
            char* argv = (char*)nodeName.c_str();
            if(!ros::isInitialized()) {
                ros::init(argc, &argv, nodeName);
            }
            ros::NodeHandle n;
            Esiaf_Handler::initialize_esiaf(&n, nodeDesignation);
        }
    };
}


using namespace boost::python;

BOOST_PYTHON_MODULE(pyesiaf){
        enum_<esiaf_ros::NodeDesignation>("NodeDesignation")
                .value("VAD", esiaf_ros::NodeDesignation::VAD)
                .value("SpeechRec", esiaf_ros::NodeDesignation::SpeechRec)
                .value("SSL", esiaf_ros::NodeDesignation::SSL)
                .value("Gender", esiaf_ros::NodeDesignation::Gender)
                .value("Emotion", esiaf_ros::NodeDesignation::Emotion)
                .value("VoiceId", esiaf_ros::NodeDesignation::VoiceId)
                .value("Other", esiaf_ros::NodeDesignation::Other)
        ;

        enum_<esiaf_ros::Rate>("Rate")
                .value("RATE_8000", esiaf_ros::Rate::RATE_8000)
                .value("RATE_16000", esiaf_ros::Rate::RATE_16000)
                .value("RATE_32000", esiaf_ros::Rate::RATE_32000)
                .value("RATE_44100", esiaf_ros::Rate::RATE_44100)
                .value("RATE_48000", esiaf_ros::Rate::RATE_48000)
                .value("RATE_96000", esiaf_ros::Rate::RATE_96000)
        ;

        enum_<esiaf_ros::Bitrate>("Bitrate")
                .value("BIT_INT_8_SIGNED", esiaf_ros::Bitrate::BIT_INT_8_SIGNED)
                .value("BIT_INT_8_UNSIGNED", esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED)
                .value("BIT_INT_16_SIGNED", esiaf_ros::Bitrate::BIT_INT_16_SIGNED)
                .value("BIT_INT_16_UNSIGNED", esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED)
                .value("BIT_INT_24_SIGNED", esiaf_ros::Bitrate::BIT_INT_24_SIGNED)
                .value("BIT_INT_24_UNSIGNED", esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED)
                .value("BIT_INT_32_SIGNED", esiaf_ros::Bitrate::BIT_INT_32_SIGNED)
                .value("BIT_INT_32_UNSIGNED", esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED)
                .value("BIT_FLOAT_32", esiaf_ros::Bitrate::BIT_FLOAT_32)
                .value("BIT_FLOAT_64", esiaf_ros::Bitrate::BIT_FLOAT_64)
        ;

        enum_<esiaf_ros::Endian>("Endian")
                .value("LittleEndian", esiaf_ros::Endian::LittleEndian)
                .value("BigEndian", esiaf_ros::Endian::BigEndian)
        ;

        class_<esiaf_ros::PyEsiaf_Handler>("Esiaf_Handler")
                .def("initialize_esiaf", &esiaf_ros::PyEsiaf_Handler::initialize_wrapper)
                .def("start_esiaf", &esiaf_ros::Esiaf_Handler::start_esiaf)
                .def("add_input_topic", &esiaf_ros::Esiaf_Handler::add_input_topic)
                .def("add_output_topic", &esiaf_ros::Esiaf_Handler::add_output_topic)
                .def("publish", &esiaf_ros::Esiaf_Handler::publish)
                .def("add_vad_finished_callback", &esiaf_ros::Esiaf_Handler::add_vad_finished_callback)
                .def("set_vad_finished", &esiaf_ros::Esiaf_Handler::set_vad_finished)
                .def("quit_esiaf", &esiaf_ros::Esiaf_Handler::quit_esiaf)
        ;

        class_<esiaf_ros::EsiafAudioFormat>("EsiafAudioFormat")
            .def_readwrite("rate", &esiaf_ros::EsiafAudioFormat::rate)
            .def_readwrite("bitrate", &esiaf_ros::EsiafAudioFormat::bitrate)
            .def_readwrite("channels", &esiaf_ros::EsiafAudioFormat::channels)
            .def_readwrite("endian", &esiaf_ros::EsiafAudioFormat::endian)
        ;

        class_<esiaf_ros::EsiafAudioTopicInfo>("EsiafAudioTopicInfo")
                .def_readwrite("topic", &esiaf_ros::EsiafAudioTopicInfo::topic)
                .def_readwrite("allowedFormat", &esiaf_ros::EsiafAudioTopicInfo::allowedFormat)
        ;
};