cmake_minimum_required(VERSION 2.8.3)
project(heimdall_image_handler)

find_package(catkin REQUIRED COMPONENTS rospy roscpp sensor_msgs std_msgs image_transport cv_bridge heimdall_msgs)
find_package(Boost REQUIRED COMPONENTS
  serialization
)
find_package(OpenCV REQUIRED)

add_definitions("-std=c++11 -O3 -s")

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES heimdall_image_handler
    CATKIN_DEPENDS cv_bridge
    DEPENDS opencv_imgproc
)

include_directories(include)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

add_executable(ih_test src/test.cc)

add_executable(image_handler 
    src/main.cc
    src/image_handler/constructor.cc
    src/image_handler/disableMask.cc
    src/image_handler/validateMask.cc
    src/image_handler/setRGBTopic.cc
    src/image_handler/checkPath.cc
    src/image_handler/activityCallback.cc
    src/image_handler/volumeCallback.cc
    src/image_handler/setRateAdaptive.cc
    src/image_handler/enableMask.cc
    src/image_handler/setDepthEnabled.cc
    src/image_handler/updateMask.cc
    src/image_handler/updateLastFrame.cc
    src/image_handler/destructor.cc
    src/image_handler/commandService.cc
    src/image_handler/depthCallback.cc
    src/image_handler/setDepthTopic.cc
    src/image_handler/rgbCallback.cc
    src/image_handler/captureRGB.cc
    src/image_handler/setRGBEnabled.cc
    src/image_handler/check.cc
    src/image_handler/format.cc
    src/image_handler/storeVideoRequest.cc
    src/image_handler/generateVideo.cc
    src/image_handler/stampImage.cc
)

add_dependencies(image_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ih_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(image_handler
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

target_link_libraries(ih_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
