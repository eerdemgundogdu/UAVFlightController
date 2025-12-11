cmake_minimum_required(VERSION 3.16)
project(UAVVision CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(CUDA)

add_executable(uav_vision
    src/main.cpp
    src/object_detector.cpp
    src/visual_odometry.cpp
    src/target_tracker.cpp
    src/serial_comm.cpp
)

target_include_directories(uav_vision PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(uav_vision
    ${OpenCV_LIBS}
    pthread
)

if(CUDA_FOUND)
    target_compile_definitions(uav_vision PRIVATE USE_CUDA)
    target_link_libraries(uav_vision ${CUDA_LIBRARIES})
endif()
