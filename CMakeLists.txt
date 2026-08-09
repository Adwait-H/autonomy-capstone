cmake_minimum_required(VERSION 3.10)
project(navio2sas)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(SOURCES
    ./sas.cpp
    ./datalink/network.cpp

    ${CMAKE_SOURCE_DIR}/MLX90640_API.cpp
    ${CMAKE_SOURCE_DIR}/MLX90640_LINUX_I2C_Driver.cpp
    ${CMAKE_SOURCE_DIR}/datalink/network.cpp
)

# Library sources
set(MLX90640_API_SOURCES
    ${CMAKE_SOURCE_DIR}/MLX90640_API.cpp
    ${CMAKE_SOURCE_DIR}/MLX90640_LINUX_I2C_Driver.cpp
    ${CMAKE_SOURCE_DIR}/datalink/network.cpp
)

# Create the static library
add_library(MLX90640_API STATIC ${MLX90640_API_SOURCES})


# Add the executable
add_executable(navio2sas ${SOURCES})
target_compile_options(navio2sas PRIVATE -Wno-address-of-packed-member)

target_include_directories(navio2sas PRIVATE
    ${CMAKE_SOURCE_DIR}/Navio
    ${CMAKE_SOURCE_DIR}/Navio/Common
    ${CMAKE_SOURCE_DIR}/Navio/Navio+
    ${CMAKE_SOURCE_DIR}/Navio/Navio2
)

# Include directory
target_include_directories(MLX90640_API PUBLIC
    ${CMAKE_SOURCE_DIR}
)

# Executable)
add_executable(main
    ${CMAKE_SOURCE_DIR}/mlx90640_sender.cpp
)

add_custom_target(
    navio_lib ALL
    COMMAND ${CMAKE_MAKE_PROGRAM}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/Navio
    COMMENT "Building Navio library using Makefile"
)

add_dependencies(navio2sas navio_lib)

# Link the Navio library and other required libraries
target_link_libraries(navio2sas
    PRIVATE
    ${CMAKE_SOURCE_DIR}/Navio/libnavio.a
    pthread
    rt
    pigpio
    atomic

    MLX90640_API
)

# Add compiler flags for optimization and vectorization
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    execute_process(COMMAND nproc OUTPUT_VARIABLE NPROC OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -flto=${NPROC} -Wall -Wextra -w -g -fno-omit-frame-pointer -O0")
    # target_link_options(navio2sas PRIVATE -fsanitize=address)
    # -O0: no optimization
    # -march=native: Optimize for the host machine's architecture
    # -ffast-math: Allow aggressive floating-point optimizations (be cautious with this flag)
    # -flto: Perform link-time optimization
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /arch:AVX2")
    # /O2: Optimize for speed
    # /arch:AVX2: Enable AVX2 instruction set for vectorization
endif()

# Enable parallel LTO
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    execute_process(COMMAND nproc OUTPUT_VARIABLE NPROC OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(CMAKE_JOB_POOLS link=${NPROC})
    set(CMAKE_JOB_POOL_LINK link)
endif()


# THERMAL CAMERA STUFF

# main uses the MLX90640_API library
target_link_libraries(main
    PRIVATE
    MLX90640_API
)