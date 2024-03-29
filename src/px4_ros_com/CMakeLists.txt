cmake_minimum_required(VERSION 3.5)

project(px4_ros_com)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Check if any sanitizers set
include(EnableSanitizers)

# Check if ROS_DISTRO is greater or equal to Dashing
# Required since
#   - "create_subscription()" and "create_publisher()" APIs changed
#   - eigen3_cmake_module is only available in these distros
list(APPEND ROS_DISTROS "dashing" "eloquent" "foxy" "galactic" "rolling")
set(ROS_DISTRO)
if(DEFINED ENV{ROS2_DISTRO})
  set(ROS_DISTRO $ENV{ROS2_DISTRO})
  if(${ROS_DISTRO} IN_LIST ROS_DISTROS)
    add_definitions(-DROS_DEFAULT_API=1)
  endif()
elseif(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
  if(${ROS_DISTRO} IN_LIST ROS_DISTROS)
    add_definitions(-DROS_DEFAULT_API=1)
  endif()
else()
  message(FATAL_ERROR "No ROS Distro set")
endif()

set(USES_DEPRECATED_EXPORT_API "ardent" "bouncy" "crystal" "dashing" "eloquent")

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(rclcpp REQUIRED)
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(dynamixel_sdk_examples REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)

find_package(PythonInterp 3 REQUIRED)
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  find_package(eigen3_cmake_module REQUIRED)
endif()
find_package(Eigen3 REQUIRED)

###################################
# Generate micro-RTPS agent code ##
###################################

get_filename_component(PX4_MSGS_DIR "../" ABSOLUTE BASE_DIR ${px4_msgs_DIR})
set(MSGS_DIR "${PX4_MSGS_DIR}/msg" CACHE INTERNAL "MSGS_DIR")
include(GenerateMicroRTPSAgent)

#################
# Setup targets #
#################

include_directories(include)
include_directories(${ARMADILLO_INCLUDE_DIRS})

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    rclcpp
    px4_msgs
    geometry_msgs
    Armadillo
    Eigen3
    LAPACK
    BLAS
    dynamixel_sdk
    dynamixel_sdk_custom_interfaces
    dynamixel_sdk_examples
  )
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

#target_link_libraries(${target} ${ARMADILLO_LIBRARIES} Eigen3::Eigen ${geometry_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES})

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Armadillo LAPACK BLAS Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(frame_transforms ${ARMADILLO_LIBRARIES} Eigen3::Eigen ${geometry_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES})

# Add microRTPS agent
add_executable(micrortps_agent ${MICRORTPS_AGENT_FILES})
ament_target_dependencies(micrortps_agent fastcdr fastrtps rclcpp)
target_link_libraries(micrortps_agent fastcdr fastrtps ${rclcpp_LIBRARIES})

# Add examples
custom_executable(examples/listeners ramping_function)
custom_executable(examples/listeners sensor_integrator_INDI)
custom_executable(examples/listeners CA_INDI_POS)
custom_executable(examples/listeners servo_publisher)

############
# Install ##
############

# Export information to downstream packages
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)
else()
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime Eigen3 px4_msgs geometry_msgs sensor_msgs)
endif()
if (ROS_DISTRO IN_LIST USES_DEPRECATED_EXPORT_API)
  ament_export_interfaces(export_frame_transforms HAS_LIBRARY_TARGET)
else()
  ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)
endif()

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)

# Install artifacts
install(DIRECTORY cmake
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS frame_transforms
        EXPORT export_frame_transforms
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(TARGETS micrortps_agent
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

############
# Testing ##
############

# Install tests
install(DIRECTORY
  test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
