cmake_minimum_required(VERSION 3.16)

# BSP component is resolved via main/idf_component.yml (local path).
# Do not add EXTRA_COMPONENT_DIRS pointing to the BSP parent.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(hello_audio_loopback)