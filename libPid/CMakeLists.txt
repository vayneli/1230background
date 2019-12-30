include_directories(./include)
AUX_SOURCE_DIRECTORY(./src Pid_SRCS)
ADD_LIBRARY(Pid ${Pid_SRCS})