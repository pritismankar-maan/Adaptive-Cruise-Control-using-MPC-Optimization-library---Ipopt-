cmake_minimum_required(VERSION 3.13)

project(ACC)

include_directories("/usr/local/include")
link_directories("/usr/local/lib")


add_executable(acc main.cpp)
target_link_libraries(acc ipopt nlopt)
install(TARGETS acc DESTINATION bin)