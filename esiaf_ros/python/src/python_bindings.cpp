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
#include <boost/version.hpp>

#if BOOST_VERSION < 106300
    #include <boost/numpy.hpp>
#else
    #include <boost/python/numpy.hpp>
#endif

// std includes
#include <string>
#include <functional>
#include <sox.h>

// some namespaces to reduce obfuscation
namespace bp = boost::python;
#if BOOST_VERSION < 106300
    namespace np = boost::numpy;
#else
    namespace np = boost::python::numpy;
#endif

namespace mp = moveit::py_bindings_tools;

namespace esiaf_ros {

    /**
     * This class will wrap some functionality for the PyEsiaf_Handler which needs to happen between superclass
     * Constructor calls.
     */
    class PyInit {
    protected:
        PyInit(std::string nodeName) {
            Py_Initialize();
            //np::initialize();
            n = new ros::NodeHandle();
        }

        ~PyInit() {
            delete n;
        }

        ros::NodeHandle *n;
    };

    /**
     * This class is necessary, because rospy.init() and ros::init() are distince from one another and rospy has no method
     * of obtaining the ros::NodeHandle necessary to initialize the EsiafHandler.
     */
    class PyEsiaf_Handler :
            protected mp::ROScppInitializer,
            protected PyInit,
            public Esiaf_Handler {
    public:

        PyThreadState *mainThreadState;

        PyEsiaf_Handler(std::string nodeName,
                        NodeDesignation nodeDesignation,
                        boost::python::list &arglist) :
                mp::ROScppInitializer(nodeName, arglist),
                PyInit(nodeName),
                Esiaf_Handler(n, nodeDesignation) {

            Py_Initialize();
            ROS_INFO("py init");
            PyEval_InitThreads();
            ROS_INFO("py threads");
            mainThreadState = PyThreadState_Get();
            ROS_INFO("py threadstate get");
            //PyEval_ReleaseLock();
            ROS_INFO("py lock released");

        };

        void publish_wrapper(std::string topic,
                             np::ndarray signal,
                             const std::string &timeStamps) {
            size_t size = signal.shape(0) * signal.get_dtype().get_itemsize();
            int8_t *data = (int8_t *) signal.get_data();
            std::vector <int8_t> signal_in_vec(data, data + size);

            esiaf_ros::RecordingTimeStamps timeStamps_cpp;
            mp::deserializeMsg(timeStamps, timeStamps_cpp);
            Esiaf_Handler::publish(topic, signal_in_vec, timeStamps_cpp);
        }

        void quit_wrapper() {
            Esiaf_Handler::quit_esiaf();
        }

        static np::ndarray convert_vec_to_ndarray(EsiafAudioTopicInfo &input, const std::vector <int8_t> &audio) {
            esiaf_ros::Bitrate info = esiaf_ros::Bitrate(input.allowedFormat.bitrate);
            np::dtype d_type = np::dtype::get_builtin<int>();
            size_t datatype_size;
            switch (info) {
                case esiaf_ros::Bitrate::BIT_INT_8_UNSIGNED: {
                    d_type = np::dtype::get_builtin<uint8_t>();
                    datatype_size = sizeof(uint8_t);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_8_SIGNED: {
                    d_type = np::dtype::get_builtin<int8_t>();
                    datatype_size = sizeof(int8_t);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_16_UNSIGNED: {
                    d_type = np::dtype::get_builtin<uint16_t>();
                    datatype_size = sizeof(uint16_t);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_16_SIGNED: {
                    d_type = np::dtype::get_builtin<int16_t>();
                    datatype_size = sizeof(int16_t);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_24_UNSIGNED: {
                    d_type = np::dtype::get_builtin<sox_uint24_t>();
                    datatype_size = sizeof(sox_uint24_t);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_24_SIGNED: {
                    d_type = np::dtype::get_builtin<sox_int24_t>();
                    datatype_size = sizeof(sox_int24_t);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_32_UNSIGNED: {
                    d_type = np::dtype::get_builtin<uint32_t>();
                    datatype_size = sizeof(uint32_t);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_INT_32_SIGNED: {
                    d_type = np::dtype::get_builtin<int32_t>();
                    datatype_size = sizeof(int32_t);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_FLOAT_32: {
                    d_type = np::dtype::get_builtin<float_t>();
                    datatype_size = sizeof(float_t);
                    break;
                }
                case esiaf_ros::Bitrate::BIT_FLOAT_64: {
                    d_type = np::dtype::get_builtin<double_t>();
                    datatype_size = sizeof(double_t);
                    break;
                }
                default:
                    std::string ex_text = "Pyesiaf: Bitrate is not supported: " + std::to_string(int(info));
                    throw std::invalid_argument(ex_text);
            }

            ROS_INFO("dt prepared");
            bp::tuple shape = bp::make_tuple((sizeof(int8_t) * audio.size()) / datatype_size);
            ROS_INFO("shape created");
            bp::tuple stride = bp::make_tuple(datatype_size);
            ROS_INFO("stride created");

            bp::object own;
            ROS_INFO("own created");
            np::ndarray output = np::from_data(&audio[0], d_type, shape, stride, own);
            ROS_INFO("output created");
            return output;
        }


        void add_input_topic_wrapper(EsiafAudioTopicInfo &input,
                                     boost::function<void(np::ndarray, esiaf_ros::RecordingTimeStamps)> callback) {

            //np::ndarray foo = np::zeros(bp::make_tuple(3, 3), np::dtype::get_builtin<double>());
            //std::cout << "foo shape(0) not labda " << foo.shape(0) << std::endl << std::flush;
            auto callback_fun = [&](const std::vector <int8_t> &audio,
                                    const esiaf_ros::RecordingTimeStamps &recordingTimeStamps) {


                //PyEval_AcquireLock();

                PyInterpreterState * mainInterpreterState = mainThreadState->interp;
                // create a thread state object for this thread
                PyThreadState * myThreadState = PyThreadState_New(mainInterpreterState);


                PyThreadState_Swap(myThreadState);
                //ROS_INFO("numpy initialize finished");

                np::ndarray output = convert_vec_to_ndarray(input, audio);


                callback(output, recordingTimeStamps);
                std::cout << "after python callback call" << std::endl << std::flush;
                PyThreadState_Swap(NULL);
                //PyEval_ReleaseLock();
                std::cout << "callback_fun finished" << std::endl << std::flush;
            };

            Esiaf_Handler::add_input_topic(input, callback_fun);
        }

        void operator=(PyEsiaf_Handler const &) = delete;  // delete the copy-assignment operator
    };
}


BOOST_PYTHON_MODULE(pyesiaf){

        np::initialize();

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
