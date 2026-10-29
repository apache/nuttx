# ##############################################################################
# boards/xtensa/esp32s2/common/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

add_subdirectory(src)
target_include_directories(board PRIVATE include)

# Linker scripts, as the boards' scripts/Make.defs

set_property(
  GLOBAL APPEND
  PROPERTY LD_SCRIPT
           ${CMAKE_CURRENT_SOURCE_DIR}/scripts/esp32s2_aliases.ld
           ${CMAKE_CURRENT_SOURCE_DIR}/scripts/esp32s2_peripherals.ld
           ${CMAKE_CURRENT_SOURCE_DIR}/scripts/flat_memory.ld
           ${CMAKE_CURRENT_SOURCE_DIR}/scripts/esp32s2_sections.ld)
