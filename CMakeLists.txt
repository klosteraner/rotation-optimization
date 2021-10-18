cmake_minimum_required(VERSION 3.8)
project(rotation-optimization)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

# Some settings

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(DEBUG_LOGGING 0) # 0: Off, 1: On
if(DEBUG_LOGGING)
  message("[rotation-optimization] Compile with extra logging")
endif()

# App configuration

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

target_compile_definitions(app PRIVATE DEBUG_LOGGING=${DEBUG_LOGGING})

target_link_libraries(app ${CONAN_LIBS})
