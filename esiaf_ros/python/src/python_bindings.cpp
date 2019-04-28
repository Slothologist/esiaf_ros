//
// Created by rfeldhans on 08.04.19.
//


#include <string>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "../include/roscpp_initializer.h"
#include "../include/serialize_msg.h"
#include "ros/ros.h"
#include "esiaf_ros/RecordingTimeStamps.h"
#include "../../include/esiaf_ros.h"


namespace bp = boost::python;
namespace mp = moveit::py_bindings_tools;

namespace esiaf_ros {

    /**
     * This class is necessary, because rospy.init() and ros::init() are distince from one another and rospy has no method
     * of obtaining the ros::NodeHandle necessary to initialize the EsiafHandler.
     */
    class PyEsiaf_Handler :
            protected mp::ROScppInitializer,
            public Esiaf_Handler {
    public:

        PyEsiaf_Handler(std::string nodeName,
                        NodeDesignation nodeDesignation,
                        boost::python::list& arglist):
                mp::ROScppInitializer(nodeName, arglist),
                n(new ros::NodeHandle()),
                Esiaf_Handler(n,nodeDesignation)
        {
        };

        void publish_wrapper(std::string topic,
                             boost::python::numpy::ndarray signal,
                             const std::string& timeStamps){
            std::cout << "publish" << std::endl;
            size_t size = signal.shape(0) * signal.get_dtype().get_itemsize();
            std::cout << "signal-shape: " << signal.shape(0) << std::endl;
            std::cout << "itemsize: " << signal.get_dtype().get_itemsize() << std::endl;
            std::cout << "arr-size: " << size << std::endl;
            int8_t * data = (int8_t*) signal.get_data();
            std::vector<int8_t> signal_in_vec(data, data + size);

            esiaf_ros::RecordingTimeStamps timeStamps_cpp;
            mp::deserializeMsg(timeStamps, timeStamps_cpp);
            Esiaf_Handler::publish(topic, signal_in_vec, timeStamps_cpp);
        }

        void quit_wrapper(){
            delete n;
            Esiaf_Handler::quit_esiaf();
        }

    private:
        ros::NodeHandle* n;
    };
}


BOOST_PYTHON_MODULE(pyesiaf){

        bp::enum_<esiaf_ros::NodeDesignation>("NodeDesignation")
                .value("VAD", esiaf_ros::NodeDesignation::VAD)
                .value("SpeechRec", esiaf_ros::NodeDesignation::SpeechRec)
                .value("SSL", esiaf_ros::NodeDesignation::SSL)
                .value("Gender", esiaf_ros::NodeDesignation::Gender)
                .value("Emotion", esiaf_ros::NodeDesignation::Emotion)
                .value("VoiceId", esiaf_ros::NodeDesignation::VoiceId)
                .value("Other", esiaf_ros::NodeDesignation::Other)
        ;

        bp::enum_<esiaf_ros::Rate>("Rate")
                .value("RATE_8000", esiaf_ros::Rate::RATE_8000)
                .value("RATE_16000", esiaf_ros::Rate::RATE_16000)
                .value("RATE_32000", esiaf_ros::Rate::RATE_32000)
                .value("RATE_44100", esiaf_ros::Rate::RATE_44100)
                .value("RATE_48000", esiaf_ros::Rate::RATE_48000)
                .value("RATE_96000", esiaf_ros::Rate::RATE_96000)
        ;

        bp::enum_<esiaf_ros::Bitrate>("Bitrate")
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

        bp::enum_<esiaf_ros::Endian>("Endian")
                .value("LittleEndian", esiaf_ros::Endian::LittleEndian)
                .value("BigEndian", esiaf_ros::Endian::BigEndian)
        ;

        bp::class_<esiaf_ros::EsiafAudioFormat>("EsiafAudioFormat")
            .def_readwrite("rate", &esiaf_ros::EsiafAudioFormat::rate)
            .def_readwrite("bitrate", &esiaf_ros::EsiafAudioFormat::bitrate)
            .def_readwrite("channels", &esiaf_ros::EsiafAudioFormat::channels)
            .def_readwrite("endian", &esiaf_ros::EsiafAudioFormat::endian)
        ;

        bp::class_<esiaf_ros::EsiafAudioTopicInfo>("EsiafAudioTopicInfo")
                .def_readwrite("topic", &esiaf_ros::EsiafAudioTopicInfo::topic)
                .def_readwrite("allowedFormat", &esiaf_ros::EsiafAudioTopicInfo::allowedFormat)
        ;

        bp::class_<esiaf_ros::PyEsiaf_Handler>("Esiaf_Handler", bp::init<std::string, esiaf_ros::NodeDesignation, bp::list&>())
        .def("start_esiaf", &esiaf_ros::PyEsiaf_Handler::start_esiaf)
        .def("quit_esiaf", &esiaf_ros::PyEsiaf_Handler::quit_wrapper)
        .def("set_vad_finished", &esiaf_ros::PyEsiaf_Handler::set_vad_finished)
        .def("add_output_topic", &esiaf_ros::PyEsiaf_Handler::add_output_topic)
        .def("publish", &esiaf_ros::PyEsiaf_Handler::publish_wrapper)
        .def("add_input_topic", &esiaf_ros::PyEsiaf_Handler::add_input_topic)
        .def("add_vad_finished_callback", &esiaf_ros::PyEsiaf_Handler::add_vad_finished_callback)
        ;
};
