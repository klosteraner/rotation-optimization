cmake_minimum_required(VERSION 2.4)
project(rotation-optimization)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

add_executable(app
  src/main.cpp

  src/geometry.hpp
  src/geometry.cpp
  src/optimization.cpp
  src/optimization.h
  src/reprojectionError.hpp
  src/setupScene.cpp
  src/setupScene.h
  src/types.h)
target_link_libraries(app ${CONAN_LIBS})
