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

// some namespaces and typedefs to reduce obfuscation
namespace bp = boost::python;
#if BOOST_VERSION < 106300
    namespace np = boost::numpy;
#else
    namespace np = boost::python::numpy;
#endif

namespace mp = moveit::py_bindings_tools;

typedef std::shared_ptr<bp::object> input_func_pointer;

namespace esiaf_ros {

    /**
     * This class will wrap some functionality for the PyEsiaf_Handler which needs to happen between superclass
     * Constructor calls.
     */
    class PyInit {
    protected:
        PyInit(std::string nodeName) {
            n = new ros::NodeHandle();
        }

        ~PyInit() {
            delete n;
        }

        ros::NodeHandle *n;
    };

    class python_gil
    {
    public:
        python_gil()  { state_ = PyGILState_Ensure(); }
        ~python_gil() { PyGILState_Release(state_);   }
    private:
        PyGILState_STATE state_;
    };


    /**
     * This class is necessary, because rospy.init() and ros::init() are distinct from one another and rospy has no method
     * of obtaining the ros::NodeHandle necessary to initialize the EsiafHandler.
     */
    class PyEsiaf_Handler :
            protected mp::ROScppInitializer,
            protected PyInit,
            public Esiaf_Handler {
    public:
        std::map<std::string, input_func_pointer> input_callbacks;
        std::map<std::string, input_func_pointer> input_vad_callbacks;
        std::map<std::string, input_func_pointer> input_ssl_callbacks;


        PyEsiaf_Handler(std::string nodeName,
                        NodeDesignation nodeDesignation,
                        boost::python::list &arglist) :
                mp::ROScppInitializer(nodeName, arglist),
                PyInit(nodeName),
                Esiaf_Handler(n, nodeDesignation) {

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
            int channels = input.allowedFormat.channels;
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

            ROS_DEBUG("dt prepared");
            bp::tuple shape;
            bp::tuple stride;
            int actual_num_frames = (sizeof(int8_t) * audio.size()) / datatype_size;
            if(channels == 1){
                shape = bp::make_tuple(actual_num_frames);
                stride = bp::make_tuple(datatype_size);
            } else{
                shape = bp::make_tuple(actual_num_frames/channels, channels);
                stride = bp::make_tuple(channels*datatype_size, datatype_size);
            }
            ROS_DEBUG("shape created");
            ROS_DEBUG("stride created");

            bp::object own;
            ROS_DEBUG("own created");
            np::ndarray output = np::from_data(&audio[0], d_type, shape, stride, own);
            ROS_DEBUG("output created");
            return output;
        }


        void add_input_topic_wrapper(EsiafAudioTopicInfo &input,
                                     bp::object callback){
            // save callback function. this is necessary because callback will get out of scope and not be
            // available anymore in the lambda below
            input_func_pointer pointy(new bp::object(callback));
            input_callbacks[input.topic] = pointy;

            auto callback_fun = [&](const std::vector <int8_t> &audio,
                                    const esiaf_ros::RecordingTimeStamps &recordingTimeStamps) {
                python_gil lock;

                // convert both arguments to python types
                np::ndarray output = convert_vec_to_ndarray(input, audio);
                ROS_DEBUG("lambda output build");
                std::string serialized_timeStamps = mp::serializeMsg(recordingTimeStamps);
                bp::str bp_timeStamps = bp::str(serialized_timeStamps);

                try {
                    // call python callback funtion
                    auto callback_internal = input_callbacks[input.topic];
                    bp::str d = bp::extract<bp::str>((*callback_internal).attr("__class__").attr("__name__"));
                    std::string stringo = bp::extract<std::string>(d);
                    ROS_DEBUG("class in callback function: %s",stringo.c_str());

                    (*callback_internal)(output, bp_timeStamps);
                    ROS_DEBUG("callback done");
                } catch (const bp::error_already_set& ) {
                    ROS_INFO("error while calling callback function");

                    PyErr_Print();
                    bp::object attributeError = bp::import("exceptions").attr("AttributeError");
                    PyObject *e, *v, *t;
                    PyErr_Fetch(&e, &v, &t);

                    // A NULL e means that there is not available Python
                    // exception
                    if (!e){
                        PyErr_Print();
                        ROS_INFO("no additional error message");
                        return;
                    }


                    // We didn't do anything with the Python exception,
                    // and we never took ownership of the refs, so it's
                    // safe to simply pass them back to PyErr_Restore
                    PyErr_Restore(e, v, t);
                }
                ROS_DEBUG("after python callback call");
            };

            Esiaf_Handler::add_input_topic(input, callback_fun);
        }

        void add_vad_finished_callback_wrapper(EsiafAudioTopicInfo &input,
                                      bp::object callback){
            // save callback function. this is necessary because callback will get out of scope and not be
            // available anymore in the lambda below
            input_func_pointer pointy(new bp::object(callback));
            input_vad_callbacks[input.topic] = pointy;

            auto callback_fun = [&]() {
                python_gil lock;

                // call python callback funtion
                auto callback_internal = input_vad_callbacks[input.topic];
                bp::str d = bp::extract<bp::str>((*callback_internal).attr("__class__").attr("__name__"));
                std::string stringo = bp::extract<std::string>(d);
                ROS_DEBUG("class in vad callback function: %s",stringo.c_str());

                (*callback_internal)();
                ROS_DEBUG("vad callback done");
            };
            Esiaf_Handler::add_vad_finished_callback(input, callback_fun);
        }

        void add_ssl_dir_callback_wrapper(EsiafAudioTopicInfo &input,
                                               bp::object callback) {
            // save callback function. this is necessary because callback will get out of scope and not be
            // available anymore in the lambda below
            input_func_pointer pointy(new bp::object(callback));
            input_ssl_callbacks[input.topic] = pointy;

            auto callback_fun = [&](const std::vector<esiaf_ros::SSLDir> &sslDirs) {
                python_gil lock;

                // call python callback funtion
                auto callback_internal = input_ssl_callbacks[input.topic];
                bp::str d = bp::extract<bp::str>((*callback_internal).attr("__class__").attr("__name__"));
                std::string stringo = bp::extract<std::string>(d);
                ROS_DEBUG("class in vad callback function: %s",stringo.c_str());

                (*callback_internal)(sslDirs);
                ROS_DEBUG("vad callback done");
            };
            Esiaf_Handler::add_ssl_dir_callback(input, callback_fun);
        }


        void operator=(PyEsiaf_Handler const &) = delete;  // delete the copy-assignment operator
    };
}


BOOST_PYTHON_MODULE(pyesiaf){

        Py_Initialize();
        PyEval_InitThreads();
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
                .def("add_vad_finished_callback", &esiaf_ros::PyEsiaf_Handler::add_vad_finished_callback_wrapper)
                .def("set_ssl_dirs", &esiaf_ros::PyEsiaf_Handler::set_ssl_dirs)
                .def("add_ssl_dir_callback", &esiaf_ros::PyEsiaf_Handler::add_ssl_dir_callback_wrapper)
        ;


        esiaf_ros::function_converter()
                .from_python<void(np::ndarray, bp::str)>()
                .from_python<void()>()
        ;

        wrap_roscpp_initializer();

};
