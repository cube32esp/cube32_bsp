cmake_minimum_required(VERSION 3.16)

# Point CMake directly to the BSP component for local development builds.
# When the component is published to the registry, this line can be removed.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

# Pull in ESP-IDF cmake infrastructure
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(hello_world)
