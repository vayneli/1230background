include_directories(./include)
AUX_SOURCE_DIRECTORY(./src SERIAL_DIR_SRCS)
ADD_LIBRARY(SerialPort ${SERIAL_DIR_SRCS})
target_link_libraries(SerialPort djiosdk-core ${OpenCV_LIBS})
