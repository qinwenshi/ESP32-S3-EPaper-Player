cmake_minimum_required(VERSION 3.16)

# Extra component directories (local components)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3_epaper_player)
