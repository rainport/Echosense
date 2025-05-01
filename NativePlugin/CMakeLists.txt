cmake_minimum_required(VERSION 3.12)
project(geometry_processor)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Library Paths (User needs to adjust these) ---
# Option 1: Set environment variables (e.g., SOFA_ROOT, KISSFFT_ROOT)
# Option 2: Hardcode paths (less flexible)
# Option 3: Use find_package if libraries support it (ideal but often not the case for smaller libs)

# Example using custom variables (set these via cmake command line -D... or in CMake GUI)
# Example: cmake .. -DSOFA_INCLUDE_DIR=/path/to/libmysofa/include -DSOFA_LIBRARY=/path/to/libmysofa/lib/libmysofa.a ...
set(SOFA_INCLUDE_DIR "" CACHE PATH "Path to libmysofa include directory")
set(SOFA_LIBRARY "" CACHE FILEPATH "Path to libmysofa library file (.lib, .a, .so)")
set(KISSFFT_INCLUDE_DIR "" CACHE PATH "Path to KissFFT include directory (containing kiss_fft.h)")
set(KISSFFT_SOURCE_DIR "" CACHE PATH "Path to KissFFT source directory (containing kiss_fft.c, kiss_fftr.c)") # If building KissFFT from source

# --- Find or Build Libraries ---

# libmysofa (assuming pre-built library)
if(NOT SOFA_INCLUDE_DIR OR NOT SOFA_LIBRARY)
    message(WARNING "libmysofa include directory or library file not specified. HRTF features might not build correctly.")
else()
    message(STATUS "Using libmysofa include: ${SOFA_INCLUDE_DIR}")
    message(STATUS "Using libmysofa library: ${SOFA_LIBRARY}")
    # Add an imported library target for easier linking
    add_library(mysofa STATIC IMPORTED) # Or SHARED if using .so/.dll
    set_target_properties(mysofa PROPERTIES
        IMPORTED_LOCATION "${SOFA_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${SOFA_INCLUDE_DIR}"
    )
    # Need to link against zlib if libmysofa was built with it
    find_package(ZLIB QUIET)
    if (ZLIB_FOUND)
        message(STATUS "Found ZLIB, will link against it for libmysofa.")
    else()
         message(WARNING "ZLIB not found. libmysofa might require it.")
    endif()
endif()

# KissFFT (Example: building from source)
if(NOT KISSFFT_INCLUDE_DIR OR NOT KISSFFT_SOURCE_DIR)
     message(WARNING "KissFFT include or source directory not specified. HRTF features might not build correctly.")
else()
    message(STATUS "Using KissFFT include: ${KISSFFT_INCLUDE_DIR}")
    message(STATUS "Using KissFFT source: ${KISSFFT_SOURCE_DIR}")
    # Add KissFFT source files directly to our library
    # This avoids needing to build KissFFT separately
    list(APPEND GEOMETRY_PROCESSOR_SOURCES
        "${KISSFFT_INCLUDE_DIR}/kiss_fft.c"      # Found directly in the include dir
        "${KISSFFT_INCLUDE_DIR}/tools/kiss_fftr.c" # Found in the tools subdir relative to include dir
    )
    # Define necessary KissFFT options if needed (check KissFFT docs)
    # add_definitions(-DKISS_FFT_USE_ALLOCA) # Example
    # add_definitions(-DFIXED_POINT=32) # Or adjust as needed, float is default usually
endif()


# --- Main Target ---
# Add KissFFT sources if found
list(APPEND GEOMETRY_PROCESSOR_SOURCES
    src/GeometryProcessor.cpp
)

# Create shared library
add_library(geometry_processor SHARED ${GEOMETRY_PROCESSOR_SOURCES})

# --- Include Directories ---
target_include_directories(geometry_processor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
# Add library include directories if found
if(SOFA_INCLUDE_DIR)
    target_include_directories(geometry_processor PRIVATE ${SOFA_INCLUDE_DIR})
endif()
if(KISSFFT_INCLUDE_DIR)
     target_include_directories(geometry_processor PRIVATE ${KISSFFT_INCLUDE_DIR})
endif()

# --- Linking ---
# Link against libmysofa if found
if(TARGET mysofa)
    target_link_libraries(geometry_processor PRIVATE mysofa)
    if(ZLIB_FOUND)
         target_link_libraries(geometry_processor PRIVATE ZLIB::ZLIB) # Link against ZLIB if found
    endif()
endif()

# Link against standard math library if needed (often required for sin, cos, etc.)
if(UNIX AND NOT APPLE)
    target_link_libraries(geometry_processor PRIVATE m)
endif()


# --- Platform-specific settings ---
if(APPLE)
    set_target_properties(geometry_processor PROPERTIES
        BUNDLE TRUE
        PREFIX ""
        SUFFIX ".bundle"
        # Ensure necessary frameworks are linked if required by libraries
        # Example: target_link_libraries(geometry_processor PRIVATE "-framework CoreFoundation")
    )
elseif(WIN32)
    set_target_properties(geometry_processor PROPERTIES
        PREFIX ""
        SUFFIX ".dll"
    )
    # Link against necessary Windows libraries if required (e.g., ws2_32 for networking)
else() # Linux, etc.
    set_target_properties(geometry_processor PROPERTIES
        PREFIX "lib"
        SUFFIX ".so"
    )
    # Link against pthreads if needed by libraries
    # find_package(Threads REQUIRED)
    # target_link_libraries(geometry_processor PRIVATE Threads::Threads)
endif()


# --- Optional: Installation ---
# install(TARGETS geometry_processor DESTINATION lib) # Example install rule

