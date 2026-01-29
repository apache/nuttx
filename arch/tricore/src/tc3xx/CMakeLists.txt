# ##############################################################################
# arch/tricore/src/tc3xx/CMakeLists.txt
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

set(SRCS tc3xx_timerisr.c tc3xx_serial.c)

target_sources(arch PRIVATE ${SRCS})

if(CONFIG_TRICORE_TOOLCHAIN_TASKING)
  nuttx_add_kernel_library(c_fpu)
  target_sources(c_fpu PRIVATE tc3xx_libc.c)
  target_link_options(
    nuttx PRIVATE
    --library-directory=${CMAKE_BINARY_DIR}/arch/tricore/src/exclude_chip)
endif()

set(ILLD_UNPACK "${CMAKE_CURRENT_SOURCE_DIR}/../illd/tc3xx")
set(ILLD_TARBALL "${ILLD_UNPACK}/illd.tar.gz")
set(ILLD_UNPACK_NAME "illd_release_tc3x-main")
set(ILLD_SRC "${ILLD_UNPACK}/${ILLD_UNPACK_NAME}/src/BaseSw")
set(ILLD_URL
    "https://github.com/Infineon/illd_release_tc3x/archive/refs/heads/main.tar.gz"
)

set(CONFIG_DIR "${ILLD_UNPACK}/Configurations")
set(ILLD_DIR "${ILLD_UNPACK}/Libraries/src")
set(PATCHES_DIR "${ILLD_UNPACK}/patches")

if(NOT EXISTS "${ILLD_DIR}/Libraries")
  file(MAKE_DIRECTORY "${ILLD_UNPACK}")
  file(DOWNLOAD "${ILLD_URL}" "${ILLD_TARBALL}" SHOW_PROGRESS)

  message(STATUS "Unpacking: ILLD for tc3xx")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf "${ILLD_TARBALL}"
    WORKING_DIRECTORY "${ILLD_UNPACK}"
    RESULT_VARIABLE tar_result)
  if(NOT tar_result EQUAL 0)
    message(FATAL_ERROR "Failed to unpack ${ILLD_TARBALL}")
  endif()

  file(MAKE_DIRECTORY "${ILLD_DIR}/Libraries")

  execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory "${ILLD_SRC}/"
                          "${ILLD_DIR}/Libraries/" RESULT_VARIABLE copy_result)
  if(NOT copy_result EQUAL 0)
    message(
      FATAL_ERROR
        "Failed to stage ILLD from ${ILLD_SRC} to ${ILLD_DIR}/Libraries")
  endif()

  file(REMOVE_RECURSE "${ILLD_UNPACK}/${ILLD_UNPACK_NAME}")
endif()

file(GLOB PATCH_FILES "${PATCHES_DIR}/*.patch")

foreach(PATCH ${PATCH_FILES})
  execute_process(
    COMMAND git apply --reverse --check ${PATCH}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} # Patches are applied from the repo
                                          # root
    RESULT_VARIABLE PATCH_NEEDED
    OUTPUT_QUIET ERROR_QUIET)

  if(PATCH_NEEDED EQUAL 0)
    message(STATUS "Patch ${PATCH} already applied, skipping.")
  else()
    execute_process(
      COMMAND git apply --whitespace=nowarn ${PATCH}
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} # Patches are applied from the repo
                                            # root
      RESULT_VARIABLE APPLY_RESULT)
    if(NOT APPLY_RESULT EQUAL 0)
      message(FATAL_ERROR "Failed to apply patch ${PATCH}!")
    else()
      message(STATUS "Successfully applied patch ${PATCH}.")
    endif()
  endif()
endforeach()

set(SDK_INCDIR
    "${CONFIG_DIR}"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Cpu/Std"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Pms/Std"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Stm/Std"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Scu/Std"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Src/Std"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Stm/Timer"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Cpu/Trap"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/_PinMap"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/_Impl"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/_PinMap/TC39x"
    "${ILLD_DIR}/Libraries/Infra/Platform"
    "${ILLD_DIR}/Libraries/Infra/Platform/Tricore"
    "${ILLD_DIR}/Libraries/Infra/Sfr/TC39xB"
    "${ILLD_DIR}/Libraries/Infra/Ssw/TC3xx/Tricore"
    "${ILLD_DIR}/Libraries/Service/CpuGeneric")

target_include_directories(arch PRIVATE ${SDK_INCDIR})
target_include_directories(nuttx PRIVATE ${SDK_INCDIR})

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_INCLUDE_DIRECTORIES ${SDK_INCDIR})

set(SDK_CSRCS
    "${CONFIG_DIR}/Ifx_Cfg_Ssw.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Asclin/Std/IfxAsclin.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Cpu/Std/IfxCpu.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Cpu/Trap/IfxCpu_Trap.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/_Impl/IfxAsclin_cfg.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/_Impl/IfxCpu_cfg.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/_Impl/IfxStm_cfg.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/_PinMap/TC39x/IfxAsclin_PinMap_TC39xB_LFBGA292.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Pms/Std/IfxPmsEvr.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Pms/Std/IfxPmsPm.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Port/Std/IfxPort.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Scu/Std/IfxScuCcu.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Scu/Std/IfxScuEru.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Scu/Std/IfxScuLbist.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Scu/Std/IfxScuRcu.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Scu/Std/IfxScuWdt.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Src/Std/IfxSrc.c"
    "${ILLD_DIR}/Libraries/iLLD/TC3xx/Tricore/Stm/Std/IfxStm.c"
    "${ILLD_DIR}/Libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.c"
    "${ILLD_DIR}/Libraries/Infra/Ssw/TC3xx/Tricore/Ifx_Ssw_Infra.c"
    "${ILLD_DIR}/Libraries/Infra/Ssw/TC3xx/Tricore/Ifx_Ssw_Tc0.c")

target_sources(nuttx PRIVATE ${SDK_CSRCS})

if(NOT CONFIG_TRICORE_TOOLCHAIN_TASKING)
  target_compile_options(nuttx PRIVATE -Wno-undef -Wno-strict-prototypes
                                       -Wno-shadow)
  target_compile_options(nuttx PRIVATE -fstrict-volatile-bitfields)
  target_compile_options(nuttx PRIVATE -Wno-unused-parameter
                                       -Wno-unused-but-set-parameter)
  target_compile_options(nuttx PRIVATE -Wno-implicit-fallthrough)
else()
  nuttx_add_extra_library(
    "${CMAKE_CURRENT_LIST_DIR}/illd/prebuilts/tc162/cinit.o")
endif()
