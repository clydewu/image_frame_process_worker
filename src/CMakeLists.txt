cmake_minimum_required(VERSION 2.8)

project(HOAWorker)


add_library(HOAProtobuf hoa_frame.pb.cc)
add_library(AMQPBComm AMQPBComm.cpp)
target_link_libraries(AMQPBComm HOAProtobuf /usr/local/lib/x86_64-linux-gnu/librabbitmq.so ${PROTOBUF_LDFLAGS})

add_executable(HOAWorker hoa_worker.cpp)
target_link_libraries(HOAWorker AMQPBComm ${OPENCV_LIBRARIES})
