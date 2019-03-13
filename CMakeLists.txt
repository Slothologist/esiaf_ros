cmake_minimum_required(VERSION 2.8.3)
project(esiaf_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        roscpp
        rospy
        rosnode
        message_generation
        std_msgs
        )

find_package(Boost REQUIRED)

find_package(ALSA REQUIRED)

find_library(SOXR NAMES soxr)
if (NOT SOXR)
    message(FATAL_ERROR "soxr library not found")
endif()

## Non-essential libraries:
# ros audio
find_package(audio_common_msgs QUIET)
find_package(audio_capture QUIET)
find_package(audio_play QUIET)
if (audio_common_msgs_FOUND AND audio_capture_FOUND AND audio_play_FOUND)
    message("Build with ros audio support.")
    set(ROS_AUDIO_SUPPORT 1)
else()
    message("audio common not found, build without ros audio support.")
endif()


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Set compiler flags
set (CMAKE_CXX_FLAGS "-O2")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


## Generate messages in the 'msgs' folder
add_message_files(
        DIRECTORY esiaf_ros/msgs
        FILES
        AudioFormat.msg
        AudioTopicFormatConstants.msg
        AudioTopicInfo.msg
        AugmentedAudio.msg
        ChangedConfig.msg
        EmotionInfo.msg
        EsiafRosMsg.msg
        GenderInfo.msg
        NodeInfo.msg
        RecordingTimeStamps.msg
        SpeechHypothesis.msg
        SpeechInfo.msg
        SSLDir.msg
        SSLInfo.msg
        VADInfo.msg
        VoiceIdInfo.msg
)

## Generate services in the 'srvs' folder
ADD_SERVICE_FILES(
        DIRECTORY esiaf_ros/srvs
        FILES
        RegisterNode.srv
)


generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        esiaf_ros/include
        esiaf_ros/include/nodes
        LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
)





## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(audio_segmenter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


file(GLOB LIBSOURCES "esiaf_ros/src/*.cpp")

# Declare a C++ library
add_library(esiaf_ros
        ${LIBSOURCES}
        )

## Declare C++ executables
add_executable(esiaf_grabber esiaf_ros/src/nodes/grabber.cpp esiaf_ros/src/nodes/utils.cpp)
add_executable(esiaf_record esiaf_ros/src/nodes/record.cpp esiaf_ros/src/nodes/utils.cpp)
add_executable(esiaf_play esiaf_ros/src/nodes/play.cpp esiaf_ros/src/nodes/utils.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(esiaf_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library against
target_link_libraries(esiaf_ros
        ${catkin_LIBRARIES}
        soxr
        )

## Specify libraries to link executables against
target_link_libraries(esiaf_grabber
        ${catkin_LIBRARIES}
        asound
        esiaf_ros
        )
target_link_libraries(esiaf_play
        ${catkin_LIBRARIES}
        asound
        esiaf_ros
        )
target_link_libraries(esiaf_record
        ${catkin_LIBRARIES}
        asound
        esiaf_ros
        )

## Specify include directories for the executables, so esiaf_ros.h can be found
target_include_directories(esiaf_grabber PUBLIC esiaf_ros/include/)
target_include_directories(esiaf_play PUBLIC esiaf_ros/include/)
target_include_directories(esiaf_record PUBLIC esiaf_ros/include/)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
install(TARGETS esiaf_grabber
        esiaf_play
        esiaf_record
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark libraries for installation
install(TARGETS esiaf_ros
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY ${PROJECT_NAME}/include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY ${PROJECT_NAME}/launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY ${PROJECT_NAME}/config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

CATKIN_INSTALL_PYTHON(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/esiaf_ros/scripts/start_orchestrator.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_openpose_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)