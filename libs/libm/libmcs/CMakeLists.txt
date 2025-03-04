# ##############################################################################
# libs/libm/libmcs/CMakeLists.txt
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

if(CONFIG_LIBM_LIBMCS)

  # ############################################################################
  # Config and Fetch libmcs
  # ############################################################################

  set(LIBMCS_VERSION 1.2.0)

  set(LIBMCS_URL
      https://gitlab.com/gtd-gmbh/libmcs/-/archive/${LIBMCS_VERSION}/${LIBMCS_VERSION}.zip
  )

  set(LIBMCS_DIR ${CMAKE_CURRENT_LIST_DIR}/libmcs)

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/libmcs)
    FetchContent_Declare(
      libmcs_fetch
      URL ${LIBMCS_URL} SOURCE_DIR ${LIBMCS_URL}/libmcs BINARY_DIR
          ${CMAKE_BINARY_DIR}/libs/libm/libmcs/libmcs
      PATCH_COMMAND
        patch -p2 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0001-fix-build-error-remove-unused-file-fenv.h.patch
        && patch -p2 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0002-fix-build-error-do-not-include-config.h.patch
        && patch -p2 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0003-fix-build-error-INFINITY-error-in-quickjs.c.patch
        && patch -p2 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0004-Fix-warning-function-declaration-isn-t-a-prototype-W.patch
        && patch -p2 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0005-libm-libmcs-Fix-clang-build-libmcs-warning.patch
    )

    FetchContent_GetProperties(libmcs_fetch)

    if(NOT libmcs_fetch_POPULATED)
      FetchContent_Populate(libmcs_fetch)
    endif()
    set(LIBMCS_DIR ${libmcs_fetch_SOURCE_DIR})

  endif()

  # ############################################################################
  # Flags
  # ############################################################################

  set(CFLAGS -DLIBMCS_LONG_IS_32BITS)

  # ############################################################################
  # Sources
  # ############################################################################
  file(GLOB_RECURSE CSRCS ${LIBMCS_DIR}/libm/common/*.c
       ${LIBMCS_DIR}/libm/mathd/*.c ${LIBMCS_DIR}/libm/mathf/*.c)

  set(RM_CSRCS
      ${LIBMCS_DIR}/libm/common/isfinite.c
      ${LIBMCS_DIR}/libm/common/cmplx.c
      ${LIBMCS_DIR}/libm/common/isgreater.c
      ${LIBMCS_DIR}/libm/common/isless.c
      ${LIBMCS_DIR}/libm/common/isunordered.c
      ${LIBMCS_DIR}/libm/common/isgreaterequal.c
      ${LIBMCS_DIR}/libm/common/isnan.c
      ${LIBMCS_DIR}/libm/common/isinf.c
      ${LIBMCS_DIR}/libm/common/islessequal.c
      ${LIBMCS_DIR}/libm/common/islessgreater.c
      ${LIBMCS_DIR}/libm/common/isnormal.c)

  list(REMOVE_ITEM CSRCS ${RM_CSRCS})

  # ############################################################################
  # Include Directory
  # ############################################################################

  set(INCDIR ${LIBMCS_DIR}/libm/common ${LIBMCS_DIR}/libm/mathd/internal
             ${LIBMCS_DIR}/libm/mathf/internal)

  if(CONFIG_LIBM_LIBMCS_WANT_COMPLEX)
    list(APPEND CFLAGS -DLIBMCS_WANT_COMPLEX)
    file(GLOB_RECURSE COMPLEX_CSRCS ${LIBMCS_DIR}/libm/complexd/*.c
         ${LIBMCS_DIR}/libm/complexf/*.c)
    list(APPEND CSRCS ${COMPLEX_CSRCS})
  endif()

  # ############################################################################
  # Library Configuration
  # ############################################################################

  nuttx_add_kernel_library(m SPLIT)

  target_sources(m PRIVATE ${CSRCS})
  target_include_directories(m PRIVATE ${INCDIR})
  target_compile_options(m PRIVATE ${CFLAGS})

  nuttx_create_symlink(${LIBMCS_DIR}/libm/include
                       ${CMAKE_BINARY_DIR}/include/libmcs)

endif()
