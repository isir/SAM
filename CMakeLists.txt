project(sam CXX)

cmake_minimum_required(VERSION 3.0)

set(CMAKE_VERBOSE_MAKEFILE "ON")
set(CMAKE_CXX_STANDARD 20)

add_definitions(-DMOSQUITTO_SERVER_IP="127.0.0.1"
                -DMOSQUITTO_SERVER_PORT=1883
                -DDEFAULT_CPU_CORE=2
                -DDEFAULT_THREAD_PRIO=20)

add_compile_options(-O2 -mcpu=cortex-a53 -mfloat-abi=hard)

add_link_options(-mfloat-abi=hard)

add_subdirectory(src)
