cmake_minimum_required(VERSION 3.0.2)
project(drive_chassis)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
  exterior_common_msgs
)

# logger
set(LOGGER_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/logger/include)
# set(LINK_LOGGER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/logger/lib/x86)

set(SPDLOG_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/spdlog/include)
# set(LINK_SPDLOG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/spdlog/lib/x86)

set(INI_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/ini/include)
# set(LINK_INI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/ini/lib/x86)

# set(CASADI_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/casadi/include)
# set(LINK_CASADI_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/casadi/lib/x86)


set(JSON_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/json/include)
# set(LINK_JSON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/json/lib/x86)

# set(ROADNET_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/roadnet/include)
# set(LINK_ROADNET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/roadnet/lib/x86)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM|aarch64|AARCH64")
    set(LINK_LOGGER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/logger/lib/arm)
    set(LINK_SPDLOG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/spdlog/lib/arm)
    set(LINK_INI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/ini/lib/arm)
    # set(LINK_JSON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/json/lib/arm)
else()
    set(LINK_LOGGER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/logger/lib/x86)
    set(LINK_SPDLOG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/spdlog/lib/x86)
    set(LINK_INI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/ini/lib/x86)
    # set(LINK_JSON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/json/lib/x86)
endif()

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
add_definitions("-g")
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
add_subdirectory(${PROJECT_SOURCE_DIR}/../3rd/fkyaml fkYAML)
list(APPEND ThirdLibs fkYAML::fkYAML)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_can
 CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/communication
  ${CMAKE_CURRENT_SOURCE_DIR}/datapool
  ${CMAKE_CURRENT_SOURCE_DIR}/socket_can
  ${CMAKE_CURRENT_SOURCE_DIR}/can
  ${CMAKE_CURRENT_SOURCE_DIR}/can_protocol

  # logger
  ${LOGGER_DIR}
  ${SPDLOG_DIR}
  ${INI_DIR}
  # ${CASADI_DIR}
  # ${JSON_DIR}
  # ${ROADNET_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/breakpad/include

)

link_directories(
    # logger
    ${LINK_LOGGER_DIR}
    ${LINK_SPDLOG_DIR}
    ${LINK_INI_DIR}
    # ${LINK_CASADI_DIR}
    # ${LINK_JSON_DIR}
    # ${LINK_ROADNET_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/breakpad/lib/x86

)

add_executable(drive_chassis 
  communication/src/main.cpp
  communication/src/rosbridge.cpp
  can/src/can.cpp
  socket_can/src/socket_can_transmitter.cpp

  common/config_manager/src/config_manager.cpp
)

target_link_libraries(${PROJECT_NAME} 
${catkin_LIBRARIES}

# logger
${ThirdLibs}
${LINK_LOGGER_DIR}/liblogger.a)

target_compile_definitions(drive_chassis PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
# /home/happy/programs/trackedVehicles/cszd/test1/src/chassis/src/drive_chassis
# target_compile_options(drive_chassis PRIVATE -g)
