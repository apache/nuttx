# ##############################################################################
# boards/xtensa/esp32/common/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

add_subdirectory(src)
target_include_directories(board PRIVATE include)

# Linker scripts, as the boards' scripts/Make.defs

if(CONFIG_BUILD_PROTECTED)
  set(LDSCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/protected_memory.ld
                ${CMAKE_CURRENT_SOURCE_DIR}/scripts/kernel-space.ld)
else()
  set(LDSCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/flat_memory.ld
                ${CMAKE_CURRENT_SOURCE_DIR}/scripts/esp32_sections.ld)
endif()

set_property(GLOBAL APPEND PROPERTY LD_SCRIPT ${LDSCRIPTS})

if(CONFIG_ESP32_APP_FORMAT_LEGACY)
  target_link_options(nuttx PRIVATE "SHELL:-u esp_app_desc")
endif()
