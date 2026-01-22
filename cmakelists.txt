cmake_minimum_required(VERSION 3.20)
project(Featherstone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build configuration
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Eigen and GLM
set(EIGEN3_INCLUDE_DIR "" CACHE PATH "Path to Eigen3 include directory")
set(GLM_INCLUDE_DIR "" CACHE PATH "Path to glm include directory")

# Validate Eigen path
if("${EIGEN3_INCLUDE_DIR}" STREQUAL "")
    message(FATAL_ERROR "EIGEN3_INCLUDE_DIR not set. Please provide path to Eigen3 include directory")
else()
    if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}")
        message(FATAL_ERROR "EIGEN3_INCLUDE_DIR path does not exist: ${EIGEN3_INCLUDE_DIR}")
    endif()
    if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
        message(FATAL_ERROR "EIGEN3_INCLUDE_DIR is invalid. Could not find Eigen/Core in: ${EIGEN3_INCLUDE_DIR}")
    endif()
    message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
endif()

# Validate glm path
if("${GLM_INCLUDE_DIR}" STREQUAL "")
    message(FATAL_ERROR "GLM_INCLUDE_DIR not set. Please provide path to glm include directory")
else()
    if(NOT EXISTS "${GLM_INCLUDE_DIR}")
        message(FATAL_ERROR "GLM_INCLUDE_DIR path does not exist: ${GLM_INCLUDE_DIR}")
    endif()
    if(NOT EXISTS "${GLM_INCLUDE_DIR}/glm/glm.hpp")
        message(FATAL_ERROR "GLM_INCLUDE_DIR is invalid. Could not find glm/glm.hpp in: ${GLM_INCLUDE_DIR}")
    endif()
    message(STATUS "glm found: ${GLM_INCLUDE_DIR}")
endif()

add_subdirectory(third_party)
add_subdirectory(featherstone)
add_subdirectory(renderer)
add_subdirectory(loader)
add_subdirectory(command)
# Define the shader path macro for C++
set(SCENES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/scenes/")
add_subdirectory(demos)

# Eigen type format descriptor for vs debugger
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_GLOBAL_NATVIS_FILES
    "Eigen.natvis"
)

