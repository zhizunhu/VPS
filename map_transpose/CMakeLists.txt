cmake_minimum_required(VERSION 2.8)
project(map_transpose)


find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)
add_definitions(-fPIC -shared)
cs_add_library(${PROJECT_NAME} src/map_transpose.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} )
create_console_plugin(${PROJECT_NAME})

cs_install()
cs_export()