# ##############################################################################
# boards/xtensa/esp32s3/common/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

add_subdirectory(src)
target_include_directories(board PRIVATE include)

# Linker scripts, as the boards' scripts/Make.defs

set(LDSCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/esp32s3_peripherals.ld
              ${CMAKE_CURRENT_SOURCE_DIR}/scripts/esp32s3_rom_aliases.ld)

if(CONFIG_BUILD_PROTECTED)
  list(APPEND LDSCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/protected_memory.ld
       ${CMAKE_CURRENT_SOURCE_DIR}/scripts/kernel-space.ld)
else()
  list(APPEND LDSCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/flat_memory.ld)
  if(CONFIG_ESP32S3_APP_FORMAT_LEGACY)
    list(APPEND LDSCRIPTS
         ${CMAKE_CURRENT_SOURCE_DIR}/scripts/legacy_sections.ld)
  else()
    list(APPEND LDSCRIPTS
         ${CMAKE_CURRENT_SOURCE_DIR}/scripts/esp32s3_sections.ld)
  endif()
endif()

set_property(GLOBAL APPEND PROPERTY LD_SCRIPT ${LDSCRIPTS})
