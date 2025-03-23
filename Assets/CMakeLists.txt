cmake_minimum_required(VERSION 3.12)
project(geometry_processor)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create shared library
add_library(geometry_processor SHARED
    src/GeometryProcessor.cpp
)

# Platform-specific settings
if(APPLE)
    set_target_properties(geometry_processor PROPERTIES
        BUNDLE TRUE
        PREFIX ""
        SUFFIX ".bundle"
    )
elseif(WIN32)
    set_target_properties(geometry_processor PROPERTIES
        PREFIX ""
        SUFFIX ".dll"
    )
else()
    set_target_properties(geometry_processor PROPERTIES
        PREFIX "lib"
        SUFFIX ".so"
    )
endif()

# Include directories
target_include_directories(geometry_processor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)