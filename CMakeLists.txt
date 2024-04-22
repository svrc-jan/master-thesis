cmake_minimum_required(VERSION 3.9.1)

project(MasterThesis)

set(CMAKE_CXX_STANDARD 20)
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

find_package(Curses REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


set(manual_control_file "${PROJECT_SOURCE_DIR}/src/manual_control.cpp")
set(filter_test_file "${PROJECT_SOURCE_DIR}/src/filter_test.cpp")
set(mpc_test_file "${PROJECT_SOURCE_DIR}/src/mpc_test.cpp")
set(run_model_est_file "${PROJECT_SOURCE_DIR}/src/run_model_est.cpp")
set(run_sim_file "${PROJECT_SOURCE_DIR}/src/run_sim.cpp")


message(${all_SRCS})

list(REMOVE_ITEM all_SRCS 
	${plot_test_file} 
	${manual_control_file}
	${plot_vicon_file}
	${filter_test_file}
	${mpc_test_file}
	${run_model_est_file}
	${run_sim_file}
)

add_executable(manual_control ${manual_control_file} ${all_SRCS})
target_link_libraries(manual_control ${CURSES_LIBRARIES})

add_executable(filter_test ${filter_test_file} ${all_SRCS})

add_executable(mpc_test ${mpc_test_file} ${all_SRCS})
target_link_libraries(mpc_test nlopt)
set_property(TARGET mpc_test PROPERTY CXX_STANDARD 20)

add_executable(run_model_est ${run_model_est_file} ${all_SRCS})
target_link_libraries(run_model_est ${CERES_LIBRARIES})

add_executable(run_sim ${run_sim_file} ${all_SRCS})

