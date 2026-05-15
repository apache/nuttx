# ##############################################################################
# arch/tricore/src/cmake/tasking.cmake
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

# Toolchain

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

if(CONFIG_ARCH_CHIP_FAMILY_TC3X)
  set(ARCH_SUBDIR tc3x)
elseif(CONFIG_ARCH_CHIP_FAMILY_TC4X)
  set(ARCH_SUBDIR tc4x)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/chip.cmake)

set(CMAKE_ASM_COMPILER cctc)
set(CMAKE_C_COMPILER cctc)
set(CMAKE_CXX_COMPILER cctc)
set(CMAKE_STRIP elfstrip)
set(CMAKE_OBJDUMP hldumptc)
set(CMAKE_LINKER cctc)
set(CMAKE_LD cctc)
set(CMAKE_NM nm)

# cctc takes no @file response files, the command lines fit without them

unset(CMAKE_NINJA_FORCE_RESPONSE_FILE CACHE)

# Every function and object keeps its own section, fill the alignment holes
# between them so the hex file is a few contiguous blocks.

set(CMAKE_OBJCOPY objcopy --gap-fill=0)

# Architecture flags

if(CONFIG_DEBUG_CUSTOMOPT)
  add_compile_options(${CONFIG_DEBUG_OPTLEVEL})
elseif(CONFIG_DEBUG_FULLOPT)
  add_compile_options(-O2)
endif()

add_compile_options(
  $<$<COMPILE_LANGUAGE:C>:--language=+gcc,+volatile>
  $<$<COMPILE_LANGUAGE:C>:--integer-enumeration>
  $<$<COMPILE_LANGUAGE:C>:--signed-bitfields>
  $<$<COMPILE_LANGUAGE:C>:--default-near-size=0> --fp-model=1)

# cmake-format: off
# ctc W500: expression without effect
# ctc W507: variable is possibly uninitialized
# ctc W508: empty source file
# ctc W525: discarded const qualifier at assignment
# ctc W526: enumeration constant shall be representable as int
# ctc W527: constant of type "double" saturated
# ctc W529: overflow in constant expression
# ctc W544: unreachable code
# ctc W549: condition is always true
# ctc W553: no break or comment before case label
# ctc W557: possible infinite loop
# ctc W558: struct/union/enum definition in parameter declaration
# ctc W560: possible truncation at implicit conversion
# ctc W562: unary minus applied to unsigned value
# ctc W587: underflow on constant of type "double"
# ctc W588: dead assignment eliminated
# ctc W589: pointer assumed to be nonzero - test removed
# cmake-format: on

set(TASKING_WARNINGS
    500,507,508,525,526,527,529,544,549,553,557,558,560,562,587,588,589)
add_compile_options(
  $<$<COMPILE_LANGUAGE:C>:--pass-c=--no-warnings=${TASKING_WARNINGS}>)

# NuttX brings its own startup code and C library, only the compiler helpers
# of the toolset are linked.  Duplicate code and constants are kept apart so
# that distinct objects keep distinct addresses.

add_link_options(--no-default-libraries -lfp_fpu -lrt)
add_link_options(-Wl--user-provided-initialization-code)
add_link_options(-Wl-OcLTXY)

if(CONFIG_DEBUG_LINK_MAP)
  add_link_options(-Wl--map-file=${CMAKE_BINARY_DIR}/nuttx.map)
endif()
