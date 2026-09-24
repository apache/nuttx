# ##############################################################################
# boards/xtensa/esp32/esp32-devkitc/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../common/CMakeLists.txt")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common common)
endif()

add_subdirectory(src)
