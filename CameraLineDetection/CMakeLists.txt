cmake_minimum_required(VERSION 3.10)

project(CameraLineDetection VERSION 1.2)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Set the C++-Standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(CameraLineDetection
    src/main.cpp)

# let the preprocessor know about the system name
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_definitions(CameraLineDetection PUBLIC "IS_LINUX")
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    target_compile_definitions(CameraLineDetection PUBLIC "IS_MACOS")
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_compile_definitions(CameraLineDetection PUBLIC "IS_WINDOWS")
endif()

# add_executable(OpenCVMarkerDemo
# "src/markers.cpp")
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    SET(CMAKE_GENERATOR "Win32")
    set_property(TARGET CameraLineDetection PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

    # set_property(TARGET OpenCVMarkerDemo PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()

target_link_libraries(CameraLineDetection ${OpenCV_LIBS})

# target_link_libraries( OpenCVMarkerDemo ${OpenCV_LIBS} )
set_property(TARGET CameraLineDetection PROPERTY CXX_STANDARD 17)
set_property(TARGET CameraLineDetection PROPERTY CXX_STANDARD_REQUIRED 17)