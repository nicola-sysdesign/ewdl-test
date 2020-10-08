cmake_minimum_required(VERSION 3.0.2)
project(ewdl)

add_subdirectory(SOEM)

add_compile_options(-std=c++11)

###########
## Build ##
###########
include_directories(
  include
)

add_executable(main src/main.cpp)
target_link_libraries(main soem)


message(STATUS ${CMAKE_MODULE_PATH})
