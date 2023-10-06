cmake_minimum_required(VERSION 3.9.1)

project(MasterThesis)

set(CMAKE_CXX_STANDART 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY build)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
)

file(GLOB all_SRCS
        "${PROJECT_SOURCE_DIR}/include/*.h"
        "${PROJECT_SOURCE_DIR}/include/*.hpp"
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/*.c"
        )

add_executable(master_thesis ${all_SRCS})


