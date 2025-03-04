# ##############################################################################
# libs/libm/newlib/CMakeLists.txt
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

if(CONFIG_LIBM_NEWLIB)

  # ############################################################################
  # Config and Fetch newlib
  # ############################################################################

  set(NEWLIB_VERSION 4.3.0.20230120)
  # ftp://sourceware.org/pub/newlib,newlib-4.3.0.20230120.tar.gz
  set(NEWLIB_URL
      ftp://sourceware.org/pub/newlib/newlib-${NEWLIB_VERSION}.tar.gz)

  set(NEWLIB_DIR ${CMAKE_CURRENT_LIST_DIR}/newlib)

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/newlib)
    FetchContent_Declare(
      newlib_fetch
      URL ${NEWLIB_URL} SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/newlib BINARY_DIR
          ${CMAKE_BINARY_DIR}/libs/libm/newlib/newlib
      PATCH_COMMAND
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0001-newlib-libm-fix-__RCSID-build-error.patch
        && patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0002-newlib-libm-remove-include-reent.h.patch
        && patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0003-newlib-fix-compilation-for-x86.patch &&
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0004-newlib-disable-optmisation-for-sincos.patch
    )

    FetchContent_GetProperties(newlib_fetch)

    if(NOT newlib_fetch_POPULATED)
      FetchContent_Populate(newlib_fetch)
    endif()
    set(NEWLIB_DIR ${newlib_fetch_SOURCE_DIR})

  endif()

  # ############################################################################
  # Flags
  # ############################################################################

  set(CFLAGS -Wno-undef -Wno-unused-but-set-variable -D__int32_t=int32_t
             -D__uint32_t=uint32_t -D_REENT=0 -D_REENT_THREAD_LOCAL=1)

  # ############################################################################
  # Sources
  # ############################################################################

  if(CONFIG_ARCH_ARM)
    set(ARCH_DIR arm)
  elseif(CONFIG_ARCH_ARM64)
    set(ARCH_DIR aarch64)
  elseif(CONFIG_ARCH_RISCV)
    set(ARCH_DIR riscv)
  elseif(CONFIG_ARCH_X86)
    set(ARCH_DIR i386)
  elseif(CONFIG_ARCH_X86_64)
    set(ARCH_DIR x86_64)
  elseif(CONFIG_ARCH_SPARC)
    set(ARCH_DIR sparc)
  elseif(CONFIG_ARCH_MIPS)
    set(ARCH_DIR mips)
  else()
    set(ARCH_DIR ${CONFIG_ARCH})
  endif()

  file(GLOB_RECURSE ARCH_CSRCS
       ${NEWLIB_DIR}/newlib/libm/machine/${ARCH_DIR}/*.c)

  file(GLOB_RECURSE COMMON_CSRCS ${NEWLIB_DIR}/newlib/libm/common/*.c)
  file(GLOB_RECURSE COMPLEX_CSRCS ${NEWLIB_DIR}/newlib/libm/complex/*.c)

  if(CONFIG_ARCH_X86_64)
    file(GLOB_RECURSE ARCH_CSRCS ${NEWLIB_DIR}/newlib/libm/fenv/*.c)
  endif()

  set(CSRCS ${COMMON_CSRCS} ${COMPLEX_CSRCS} ${ARCH_CSRCS})

  # aggresive optimisation can replace occurrences of sinl() and cosl() with
  # sincosl(), but sincosl() is missing in newlib which causes error. So let's
  # use custom implementation here.

  list(APPEND CSRCS ${CMAKE_CURRENT_LIST_DIR}/sincosl.c)

  if(CONFIG_LIBM_NEWLIB_HW_FP)
    file(GLOB_RECURSE MATHFP_CSRCS ${NEWLIB_DIR}/newlib/libm/mathfp/*.c)
    list(APPEND CSRCS ${MATHFP_CSRCS})
    list(
      APPEND
      CFLAGS
      -Wno-dangling-else
      -Wno-endif-labels
      -Wno-implicit-function-declaration
      -Wno-missing-braces
      -Wno-shadow
      -Wno-strict-prototypes
      -Wno-unused-const-variable)
  else()
    file(GLOB_RECURSE MATH_CSRCS ${NEWLIB_DIR}/newlib/libm/math/*.c)
    list(APPEND CSRCS ${MATH_CSRCS})
  endif()

  # ############################################################################
  # Include Directory
  # ############################################################################

  set(INCDIR ${CMAKE_CURRENT_LIST_DIR}/include ${NEWLIB_DIR}/newlib/libm/common)

  if(CONFIG_ARCH_X86_64)
    list(APPEND INCDIR ${NEWLIB_DIR}/newlib/libc/machine/shared_x86/sys)
  endif()

  # ############################################################################
  # Library Configuration
  # ############################################################################

  nuttx_add_kernel_library(m)

  target_sources(m PRIVATE ${CSRCS})
  target_include_directories(m PRIVATE ${INCDIR})
  target_compile_options(m PRIVATE ${CFLAGS})

  set_property(
    TARGET nuttx
    APPEND
    PROPERTY NUTTX_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/include)

endif()
