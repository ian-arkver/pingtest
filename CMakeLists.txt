cmake_minimum_required(VERSION 3.10)
project(pingtest VERSION 0.3.0)

find_package(Threads REQUIRED)

add_executable(pingtest pingtest.cpp)
target_include_directories(pingtest PRIVATE asio/asio/include)
target_link_libraries(pingtest ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(pingtest PRIVATE -Wall)
set_property(TARGET pingtest PROPERTY CXX_STANDARD 17)

install(TARGETS pingtest DESTINATION bin)