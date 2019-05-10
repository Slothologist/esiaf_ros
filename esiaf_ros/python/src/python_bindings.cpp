//
// Created by rfeldhans on 08.04.19.
//

// ros and roscpp includes
#include "../include/roscpp_initializer.h"
#include "../include/serialize_msg.h"
#include "../include/function_converter.h"
#include "ros/ros.h"

// esiaf includes
#include "../../include/esiaf_ros.h"
#include "esiaf_ros/RecordingTimeStamps.h"

// boost includes for ros bindings
#include <boost/python.hpp>
#include <boost/numpy.hpp>

// std includes
#include <string>
#include <functional>
#include <sox.h>

// some namespaces to reduce obfuscation
namespace bp = boost::python;
namespace np = boost::numpy;
namespace mp = moveit::py_bindings_tools;

namespace esiaf_ros {

    /**
     * This class will wrap some functionality for the PyEsiaf_Handler which needs to happen between superclass
     * Constructor calls.
     */
    class PyInit{
    protected:
        PyInit(std::string nodeName){
            Py_Initialize();
            np::initialize();
            n = new ros::NodeHandle();
        }

        ~PyInit(){
            delete n;
        }

        ros::NodeHandle* n;
    };

    /**
     * This class is necessary, because rospy.init() and ros::init() are distince from one another and rospy has no method
     * of obtaining the ros::NodeHandle necessary to initialize the EsiafHandler.
     */
    class PyEsiaf_Handler :
            protected mp::ROScppInitializer,
            protected PyInit,
            public Esiaf_Handler
    {
    public:

        PyEsiaf_Handler(std::string nodeName,
                        NodeDesignation nodeDesignation,
                        boost::python::list& arglist):
                mp::ROScppInitializer(nodeName, arglist),
                PyInit(nodeName),
                Esiaf_Handler(n, nodeDesignation)
        {
        };

        void publish_wrapper(std::string topic,
                             np::ndarray signal,
                             const std::string& timeStamps){
            size_t size = signal.shape(0) * signal.get_dtype().get_itemsize();
            int8_t * data = (int8_t*) signal.get_data();
            std::vector<int8_t> signal_in_vec(data, data + size);

            esiaf_ros::RecordingTimeStamps timeStamps_cpp;
            mp::deserializeMsg(timeStamps, timeStamps_cpp);
            Esiaf_Handler::publish(topic, signal_in_vec, timeStamps_cpp);
        }

        void quit_wrapper(){
            Esiaf_Handler::quit_esiaf();
        }

        void add_input_topic_wrapper(EsiafAudioTopicInfo &input,
                          boost::function<void( np::ndarray, esiaf_ros::RecordingTimeStamps)> callback){

            auto callback_fun = [&](const std::vector<int8_t>& audio,
                                   const esiaf_ros::RecordingTimeStamps& recordingTimeStamps){
                typedef int audiotype;
                switch (input.allowedFormat.bitrate) {
                    case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED: {
                        typedef uint8_t audiotype;
                    }
                    case esiaf_ros::Bitrate::BIT_INT_8_SIGNED: {
                        typedef int8_t audiotype;
                    }
                    case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED: {
                        typedef uint16_t audiotype;
                    }
                    case esiaf_ros::Bitrate::BIT_INT_16_SIGNED: {
                        typedef int16_t audiotype;
                    }
                    case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED: {
                        typedef sox_uint24_t audiotype;
                    }
                    case esiaf_ros::Bitrate::BIT_INT_24_SIGNED: {
                        typedef sox_int24_t audiotype;
                    }
                    case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED: {
                        typedef uint32_t audiotype;
                    }
                    case esiaf_ros::Bitrate::BIT_INT_32_SIGNED: {
                        typedef int32_t audiotype;
                    }
                    case esiaf_ros::Bitrate::BIT_FLOAT_32: {
                        typedef float_t audiotype;
                    }
                    case esiaf_ros::Bitrate::BIT_FLOAT_64: {
                        typedef double_t audiotype;
                    }
                    default:
                        std::string ex_text = "bitrate is not supported";
                        throw std::invalid_argument(ex_text);
                }

                bp::tuple shape = bp::make_tuple((sizeof(int8_t) / sizeof(audiotype)) * audio.size());
                np::dtype dt = np::dtype::get_builtin<audiotype>();
                np::ndarray ndarray = np::empty(shape, dt);

                callback(ndarray, recordingTimeStamps);
            };

            Esiaf_Handler::add_input_topic(input, callback_fun);
        }

        void operator=(PyEsiaf_Handler const &) = delete;  // delete the copy-assignment operator
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
        .def("add_input_topic", &esiaf_ros::PyEsiaf_Handler::add_input_topic_wrapper)
        .def("add_vad_finished_callback", &esiaf_ros::PyEsiaf_Handler::add_vad_finished_callback)
        ;

        esiaf_ros::function_converter()
        .from_python<void(np::ndarray, esiaf_ros::RecordingTimeStamps)>()
        .from_python<void()>()
        ;
};
