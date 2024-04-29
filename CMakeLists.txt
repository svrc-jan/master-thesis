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
set(run_sim_file "${PROJECT_SOURCE_DIR}/src/run_sim.cpp")
set(run_mpc_sim_file "${PROJECT_SOURCE_DIR}/src/run_mpc_sim.cpp")
set(run_mhe_mpc_sim_file "${PROJECT_SOURCE_DIR}/src/run_mhe_mpc_sim.cpp")
set(run_model_ident_file "${PROJECT_SOURCE_DIR}/src/run_model_ident.cpp")
set(mhe_test_file "${PROJECT_SOURCE_DIR}/src/mhe_test.cpp")



message(${all_SRCS})

list(REMOVE_ITEM all_SRCS 
	${manual_control_file}
	${filter_test_file}
	${run_sim_file}
	${run_mpc_sim_file}
	${run_mhe_mpc_sim_file}
	${run_model_ident_file}
	${mhe_test_file}
)

add_executable(manual_control ${manual_control_file} ${all_SRCS})
target_link_libraries(manual_control ${CURSES_LIBRARIES} ${CERES_LIBRARIES})

add_executable(filter_test ${filter_test_file} ${all_SRCS})

add_executable(run_sim ${run_sim_file} ${all_SRCS})

add_executable(run_mpc_sim ${run_mpc_sim_file} ${all_SRCS})
target_link_libraries(run_mpc_sim ${CERES_LIBRARIES})

add_executable(run_mhe_mpc_sim ${run_mhe_mpc_sim_file} ${all_SRCS})
target_link_libraries(run_mhe_mpc_sim ${CERES_LIBRARIES})

add_executable(run_model_ident ${run_model_ident_file} ${all_SRCS})
target_link_libraries(run_model_ident ${CERES_LIBRARIES})

add_executable(mhe_test ${mhe_test_file} ${all_SRCS})
target_link_libraries(mhe_test ${CERES_LIBRARIES})


