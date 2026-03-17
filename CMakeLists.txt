cmake_minimum_required(VERSION 3.13)
project(MPU6500_lib C CXX ASM)

add_library(MPU6500_lib STATIC
    ${CMAKE_CURRENT_LIST_DIR}/headers/MPU6500.h
    ${CMAKE_CURRENT_LIST_DIR}/headers/MPU6500.hpp
)

target_include_directories(MPU6500_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/headers
)

target_link_libraries(MPU6500_lib PUBLIC 
        hardware_spi
        hardware_i2c
        pico_stdlib
)
