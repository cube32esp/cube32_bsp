cmake_minimum_required(VERSION 3.16)

# NOTE: BSP component is resolved via main/idf_component.yml (local path).
# Do NOT add EXTRA_COMPONENT_DIRS pointing to the BSP parent — IDF scans one
# level deep and will also pick up sibling projects (e.g. cube32-app-template),
# causing a cmake script-mode crash on define_property.

# Pull in ESP-IDF cmake infrastructure
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(hello_camera)
