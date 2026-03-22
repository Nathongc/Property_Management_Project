cmake_minimum_required(VERSION 3.10)

project(final_result)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(
    include
    /usr/include/eigen3
)

add_executable(final_result src/main.cpp src/manager.cpp src/server.cpp src/owner.cpp src/grid_map.cpp src/lastar.cpp src/minisnap.cpp)

target_link_libraries(final_result
    ${OpenCV_LIBS}
    OsqpEigen::OsqpEigen)