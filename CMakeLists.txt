cmake_minimum_required(VERSION 3.9.1)

project(MasterThesis)

set(CMAKE_CXX_STANDART 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY build)

include_directories(
	${PROJECT_SOURCE_DIR}/src
)

file(GLOB_RECURSE all_SRCS
	"${PROJECT_SOURCE_DIR}/src/*.hpp"
	"${PROJECT_SOURCE_DIR}/src/*.cpp"
	"${PROJECT_SOURCE_DIR}/src/*.cc"
	"${PROJECT_SOURCE_DIR}/src/*.h"
	"${PROJECT_SOURCE_DIR}/src/*.c"
)


find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})


find_package(Curses REQUIRED)

set(plot_test_file "${PROJECT_SOURCE_DIR}/src/plot_test.cpp")
set(manual_control_file "${PROJECT_SOURCE_DIR}/src/manual_control.cpp")
set(plot_vicon_file "${PROJECT_SOURCE_DIR}/src/plot_vicon.cpp")

message(${all_SRCS})

list(REMOVE_ITEM all_SRCS 
	${plot_test_file} 
	${manual_control_file}
	${plot_vicon_file}
)


add_executable(plot_test ${plot_test_file} ${all_SRCS})
target_link_libraries(plot_test ${PYTHON_LIBRARIES})

add_executable(manual_control ${manual_control_file} ${all_SRCS})
target_link_libraries(manual_control ${PYTHON_LIBRARIES} ${CURSES_LIBRARIES})

add_executable(plot_vicon ${plot_vicon_file} ${all_SRCS})
target_link_libraries(plot_vicon ${PYTHON_LIBRARIES})

