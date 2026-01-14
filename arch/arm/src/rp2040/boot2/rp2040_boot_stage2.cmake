# ##############################################################################
# arch/arm/src/rp2040/boot2/rp2040_boot_stage2.cmake
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

# ~~~
# pico_define_boot_stage2
#
# Description: Define a boot stage 2 for the Raspberry Pi rp2040.
#
# Parameters:
#  NAME        : The name of the boot stage 2
#  path_chip   : The full path of the CMake build /${CONFIG_ARCH}/src/chip
# ~~~

function(pico_define_boot_stage2 NAME path_chip)

  set(PICO_BOOT_STAGE2_DIR "${PICO_SDK_PATH}/src/rp2040/boot_stage2")
  set(BOOT2SRC "${PICO_BOOT_STAGE2_DIR}/boot2_${CONFIG_RP2040_FLASH_CHIP}.S")

  string(REPLACE "." "-" NUTTX_BOARD_NAME "${NUTTX_BOARD}")

  set(BOOT2CFLAGSLIST
      "-T${NUTTX_BOARD_DIR}/scripts/${NUTTX_BOARD_NAME}-flash.ld"
      -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_NO_HARDWARE=0
      -DPICO_ON_DEVICE=1)

  list(APPEND BOOT2CFLAGSLIST -I${path_chip}/boot2)
  list(APPEND BOOT2CFLAGSLIST -I${PICO_BOOT_STAGE2_DIR}/asminclude)
  list(APPEND BOOT2CFLAGSLIST
       -I${PICO_SDK_PATH}/src/rp2040/hardware_regs/include)
  list(APPEND BOOT2CFLAGSLIST
       -I${PICO_SDK_PATH}/src/rp2_common/hardware_base/include)
  list(APPEND BOOT2CFLAGSLIST
       -I${PICO_SDK_PATH}/src/common/pico_base_headers/include)
  list(APPEND BOOT2CFLAGSLIST -I${PICO_SDK_PATH}/src/boards/include)
  list(APPEND BOOT2CFLAGSLIST
       -I${PICO_SDK_PATH}/src/rp2040/pico_platform/include)

  list(APPEND BOOT2CFLAGSLIST
       -I${PICO_SDK_PATH}/src/rp2_common/pico_platform_common/include)
  list(APPEND BOOT2CFLAGSLIST
       -I${PICO_SDK_PATH}/src/rp2_common/pico_platform_compiler/include)
  list(APPEND BOOT2CFLAGSLIST
       -I${PICO_SDK_PATH}/src/rp2_common/pico_platform_sections/include)
  list(APPEND BOOT2CFLAGSLIST
       -I${PICO_SDK_PATH}/src/rp2_common/pico_platform_panic/include)
  list(APPEND BOOT2CFLAGSLIST -Wl,--no-warn-rwx-segments)

  string(REPLACE ";" " " BOOT2CFLAGS "${BOOT2CFLAGSLIST}")

  set(ORIGINAL_ELF ${path_chip}/${NAME}.elf)

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E touch
            ${PICO_SDK_PATH}/src/common/pico_base_headers/include/pico/version.h
  )
  execute_process(
    COMMAND
      ${CMAKE_COMMAND} -E touch
      ${PICO_SDK_PATH}/src/common/pico_base_headers/include/pico/config_autogen.h
  )

  set(builtin
      "${CMAKE_C_COMPILER} -nostdlib ${BOOT2CFLAGS} -o ${ORIGINAL_ELF} ${BOOT2SRC}"
  )

  if(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
    execute_process(COMMAND cmd /c "${builtin}" RESULT_VARIABLE resultVar)
  else()
    execute_process(COMMAND sh -c "${builtin}" RESULT_VARIABLE resultVar)
  endif()

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E remove -f
            ${PICO_SDK_PATH}/src/common/pico_base_headers/include/pico/version.h
  )
  execute_process(
    COMMAND
      ${CMAKE_COMMAND} -E remove -f
      ${PICO_SDK_PATH}/src/common/pico_base_headers/include/pico/config_autogen.h
  )

  if(resultVar AND NOT resultVar EQUAL 0)
    message(FATAL_ERROR "Failed: ${resultVar}")
  else()
    set(ORIGINAL_BIN ${path_chip}/${NAME}.bin)
    set(PADDED_CHECKSUMMED_ASM ${path_chip}/${NAME}.S)

    execute_process(COMMAND ${CMAKE_OBJCOPY} -Obinary ${ORIGINAL_ELF}
                            ${ORIGINAL_BIN})
    execute_process(
      COMMAND ${Python3_EXECUTABLE} ${PICO_BOOT_STAGE2_DIR}/pad_checksum -s
              0xffffffff ${ORIGINAL_BIN} ${PADDED_CHECKSUMMED_ASM})
  endif()
endfunction()
