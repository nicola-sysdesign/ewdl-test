cmake_minimum_required(VERSION 3.0.2)
project(ewdl)

# SOEM
add_subdirectory(SOEM)

# C++11
add_compile_options(-std=c++11)

## Boost
find_package(Boost REQUIRED COMPONENTS
  program_options
)

###########
## Build ##
###########
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(run src/run.cpp)
add_executable(idle src/idle.cpp)

target_link_libraries(run soem ${Boost_LIBRARIES})
target_link_libraries(idle soem ${Boost_LIBRARIES})

message(STATUS ${CMAKE_MODULE_PATH})
