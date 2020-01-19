include_directories(./include)
include_directories(./MIPP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0")
#SET(OpenCV_DIR /usr/local/opencv-3.1.0/share/OpenCV)
set(CMAKE_CXX_STANDARD 14)

# arm or x86
# IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
#     SET(PLATFORM_COMPILE_FLAGS "-mfpu=neon")
# ELSE()
#     SET(PLATFORM_COMPILE_FLAGS "-march=native")

#     # some places of the algorithm are designed for 128 SIMD
#     # so 128 SSE may slightly faster than 256 AVX, you may want this
# #    SET(PLATFORM_COMPILE_FLAGS "-msse -msse2 -msse3 -msse4 -mssse3")  # SSE only
# ENDIF()

# SET(COMMON_COMPILE_FLAGS "-fopenmp -Wall -Wno-sign-compare")
SET(CMAKE_CXX_FLAGS "${PLATFORM_COMPILE_FLAGS} ${COMMON_COMPILE_FLAGS} $ENV{CXXFLAGS}")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(OpenCV  REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
AUX_SOURCE_DIRECTORY(./src Detection_SRCS)
ADD_LIBRARY(Detection ${Detection_SRCS})

target_link_libraries(Detection ${OpenCV_LIBS})
