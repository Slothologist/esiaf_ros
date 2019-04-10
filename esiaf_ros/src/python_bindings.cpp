//
// Created by rfeldhans on 08.04.19.
//



#include "../include/esiaf_ros.h"
#include <boost/python.hpp>

using namespace boost::python;

BOOST_PYTHON_MODULE(esiaf_ros){
        class_<esiaf_ros::Esiaf_Handler>("Esiaf_Handler")
                .def("start_esiaf", &esiaf_ros::Esiaf_Handler::start_esiaf)
                .def("quit_esiaf", &esiaf_ros::Esiaf_Handler::quit_esiaf)
        ;


};
