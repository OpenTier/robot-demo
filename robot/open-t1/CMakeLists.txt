cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS
    components/micro_ros_espidf_component
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/BatterySensor
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Controller
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/LedTelltale
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/MecanumDrive
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/Robot
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/SabertoothRC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/SolenoidKicker
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/WifiCom
)
project(open-t1)

set(CMAKE_C_FLAGS_RELEASE "-O2 -Oz")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Oz")
