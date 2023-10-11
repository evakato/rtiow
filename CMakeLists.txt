set(CMAKE_C_COMPILER "C:/Program Files/MinGW/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "C:/Program Files/MinGW/bin/g++.exe")
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(hello VERSION 1.0)
add_executable(hello main.cpp)
target_include_directories(hello PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)