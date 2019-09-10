include_directories(./include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0")
#SET(OpenCV_DIR /usr/local/opencv-3.1.0/share/OpenCV)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV  REQUIRED)

AUX_SOURCE_DIRECTORY(./src Detection_SRCS)
ADD_LIBRARY(Detection ${Detection_SRCS})

