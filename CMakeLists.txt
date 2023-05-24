cmake_minimum_required(VERSION 3.25)
project(arduino_test)

set(CMAKE_CXX_STANDARD 17)

include_directories(.)

add_executable(arduino_test
        arduino_new_version.ino
        arduino_old_working.ino)
