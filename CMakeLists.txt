cmake_minimum_required( VERSION 3.10 )

project(control_final)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(SOURCES ./src/main.cpp ./src/sensor/sensor.cpp ./src/model/state.cpp
            ./src/model/environment.cpp ./src/controller/controller.cpp
            ./src/controller/pid_controller.cpp ./src/model/render_config.cpp
            ./src/simulator.cpp)

include_directories(./include ./extern/moviemaker-cpp/include)

# target
add_executable(control_final ${SOURCES})


# my implementation of a raytracer
add_subdirectory(./extern/raytracer)
target_link_libraries(control_final raytracer)

# Eigen as a linear algebra library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(control_final Eigen3::Eigen)

# yaml-cpp for config files
find_package(yaml-cpp REQUIRED)
target_link_libraries(control_final yaml-cpp)

# opencv for finding the ball in the image
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( control_final ${OpenCV_LIBS} )

# Taken from https://stackoverflow.com/questions/37759833/about-magick-how-to-write-the-cmakelists
add_definitions( -DMAGICKCORE_QUANTUM_DEPTH=16 )
add_definitions( -DMAGICKCORE_HDRI_ENABLE=0 )
find_package(ImageMagick COMPONENTS Magick++)
include_directories(${ImageMagick_INCLUDE_DIRS})
target_link_libraries(control_final ${ImageMagick_LIBRARIES})
