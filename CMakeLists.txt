project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_subdirectory (src/googletest)

add_definitions(-std=c++11)
if (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
    add_definitions(-D__GLIBCXX__)
endif (APPLE)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/utils.cpp src/utils.h src/generate_path.cpp src/generate_path.h src/car_state.cpp src/car_state.h src/point.h src/point.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
include_directories(${gtest_SOURCE_DIR/include} ${gtest_SOURCE_DIR})

link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)

add_executable(path_planning_tests src/utils.cpp src/utils.h src/utils_test.cpp)
target_link_libraries(path_planning_tests gtest gtest_main)
