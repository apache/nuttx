# ##############################################################################
# arch/sim/cmake/Toolchain.cmake
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

if(APPLE)
  find_program(CMAKE_C_ELF_COMPILER x86_64-elf-gcc)
  find_program(CMAKE_CXX_ELF_COMPILER x86_64-elf-g++)
endif()

if(WIN32)
  return()
endif()

add_compile_options(-U_AIX -U_WIN32 -U__APPLE__ -U__FreeBSD__)
add_compile_options(-U__NetBSD__ -U__linux__ -U__sun__ -U__unix__)
add_compile_options(-U__ENVIRONMENT_MAC_OS_X_VERSION_MIN_REQUIRED__)

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(-g)
endif()

if(CONFIG_DEBUG_CUSTOMOPT)
  add_compile_options(${CONFIG_DEBUG_OPTLEVEL})
elseif(CONFIG_DEBUG_FULLOPT)
  add_compile_options(-O2)
endif()

if(NOT CONFIG_DEBUG_NOOPT)
  add_compile_options(-fno-strict-aliasing)
endif()

if(CONFIG_FRAME_POINTER)
  add_compile_options(-fno-omit-frame-pointer -fno-optimize-sibling-calls)
else()
  add_compile_options(-fomit-frame-pointer)
endif()

if(CONFIG_STACK_CANARIES)
  add_compile_options(-fstack-protector-all)
endif()

if(CONFIG_STACK_USAGE)
  add_compile_options(-fstack-usage)
endif()

if(CONFIG_STACK_USAGE_WARNING)
  add_compile_options(-Wstack-usage=${CONFIG_STACK_USAGE_WARNING})
endif()

if(CONFIG_ARCH_COVERAGE_ALL)
  add_compile_options(-fprofile-generate -ftest-coverage)
endif()

if(CONFIG_SCHED_GPROF_ALL OR CONFIG_SIM_GPROF)
  add_compile_options(-pg)
endif()

if(CONFIG_SIM_ASAN)
  add_compile_options(-fsanitize=address)
  add_compile_options(-fsanitize-address-use-after-scope)
  add_compile_options(-fsanitize=pointer-compare)
  add_compile_options(-fsanitize=pointer-subtract)
elseif(CONFIG_MM_KASAN_ALL)
  add_compile_options(-fsanitize=kernel-address)
endif()

if(CONFIG_SIM_UBSAN)
  add_compile_options(-fsanitize=undefined)
else()
  if(CONFIG_MM_UBSAN_ALL)
    add_compile_options(${CONFIG_MM_UBSAN_OPTION})
  endif()

  if(CONFIG_MM_UBSAN_TRAP_ON_ERROR)
    add_compile_options(-fsanitize-undefined-trap-on-error)
  endif()
endif()

if(CONFIG_ARCH_INSTRUMENT_ALL)
  add_compile_options(-finstrument-functions)
endif()

set(ARCHCFLAGS
    "-fno-common -fvisibility=hidden -ffunction-sections -fdata-sections -Wall -Wstrict-prototypes -Wshadow -Wundef -Wno-attributes -Wno-unknown-pragmas"
)
set(ARCHCXXFLAGS
    "-fno-common -nostdinc++ -fvisibility=hidden -ffunction-sections -fdata-sections -Wall -Wshadow -Wundef -Wno-attributes -Wno-unknown-pragmas"
)

if(NOT CONFIG_LIBCXXTOOLCHAIN)
  set(ARCHCXXFLAGS "${ARCHCXXFLAGS} -nostdinc++")
else()
  set(ARCHCXXFLAGS "${ARCHCXXFLAGS} -D_STDLIB_H_")
endif()

if(CONFIG_CXX_STANDARD)
  string(APPEND ARCHCXXFLAGS " -std=${CONFIG_CXX_STANDARD}")
endif()

if(NOT CONFIG_CXX_EXCEPTION)
  string(APPEND ARCHCXXFLAGS " -fno-exceptions -fcheck-new")
endif()

if(NOT CONFIG_CXX_RTTI)
  string(APPEND ARCHCXXFLAGS " -fno-rtti")
endif()

if(NOT "${CMAKE_C_FLAGS}" STREQUAL "")
  string(REGEX MATCH "${ARCHCFLAGS}" EXISTS_FLAGS "${CMAKE_C_FLAGS}")
endif()

if(NOT EXISTS_FLAGS)
  set(CMAKE_ASM_FLAGS
      "${CMAKE_ASM_FLAGS}${ARCHCFLAGS}"
      CACHE STRING "" FORCE)
  set(CMAKE_C_FLAGS
      "${CMAKE_C_FLAGS}${ARCHCFLAGS}"
      CACHE STRING "" FORCE)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS}${ARCHCXXFLAGS}"
      CACHE STRING "" FORCE)
endif()
