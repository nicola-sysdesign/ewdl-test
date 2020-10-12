cmake_minimum_required(VERSION 3.0.2)
project(ewdl)

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

add_executable(main src/main.cpp)
target_link_libraries(main soem ${Boost_LIBRARIES})


message(STATUS ${CMAKE_MODULE_PATH})
