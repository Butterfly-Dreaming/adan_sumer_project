cmake_minimum_required(VERSION 3.22)
project(VisionTest2)

set(CMAKE_CXX_STANDARD 14)

include_directories(VisionTest)

add_executable(VisionTest2
        VisionTest/armor.cpp
        VisionTest/armor.h
        VisionTest/Camera.cpp
        VisionTest/Camera.h
        VisionTest/common.cpp
        VisionTest/common.h
        VisionTest/detector.cpp
        VisionTest/detector.h
        VisionTest/main.cpp
        VisionTest/predict.cpp
        VisionTest/predict.h
        VisionTest/serialport.cpp
        VisionTest/serialport.h
        VisionTest/solveangle.cpp
        VisionTest/solveangle.h)
