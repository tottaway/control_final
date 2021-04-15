cmake_minimum_required( VERSION 3.10 )

project(control_final)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(SOURCES ./src/main.cpp ./src/sensor/sensor.cpp ./src/model/state.cpp)

include_directories(./include)

# target
# set(TESTS ./test/src/test_shapes.cpp)
add_executable(control_final ${SOURCES})


# link raytracer
add_subdirectory(./extern/raytracer)
target_link_libraries(control_final raytracer)

# external libs
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(control_final Eigen3::Eigen)
add_subdirectory(./extern/moviemaker-cpp)
target_link_libraries(control_final moviemaker-cpp)

# Taken from https://stackoverflow.com/questions/37759833/about-magick-how-to-write-the-cmakelists
add_definitions( -DMAGICKCORE_QUANTUM_DEPTH=16 )
add_definitions( -DMAGICKCORE_HDRI_ENABLE=0 )
find_package(ImageMagick COMPONENTS Magick++)
include_directories(${ImageMagick_INCLUDE_DIRS})
target_link_libraries(control_final ${ImageMagick_LIBRARIES})

