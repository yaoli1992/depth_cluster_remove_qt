cmake_minimum_required(VERSION 3.1)
project(depth_clustering_remove_qt)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


set(Boost_USE_STATIC_LIBS OFF)


find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)

find_package(PCL 1.7 REQUIRED)



include_directories(
  src
  # for QT moc files
  ${PROJECT_BINARY_DIR}/src
  # dependencies we have no control over
  SYSTEM ${EIGEN3_INCLUDE_DIR}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}

)

add_subdirectory(src)
add_subdirectory(demo)
