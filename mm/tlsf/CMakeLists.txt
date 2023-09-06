# ##############################################################################
# mm/tlsf/CMakeLists.txt
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

if(CONFIG_MM_TLSF_MANAGER)

  # ############################################################################
  # Config and Fetch tlsf
  # ########################################################################TLSF

  set(TLSF_DIR ${CMAKE_CURRENT_LIST_DIR}/tlsf)

  if(NOT EXISTS ${TLSF_DIR})
    FetchContent_Declare(
      tlsf_fetch
      GIT_REPOSITORY https://github.com/mattconte/tlsf.git
      SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/tlsf BINARY_DIR
      ${CMAKE_BINARY_DIR}/mm/tlsf/tlsf
      PATCH_COMMAND
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0001-Add-TLSF_API-and-tlsf_printf.patch &&
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0002-Define-_DEBUG-to-0-if-not-done-yet.patch
        && patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0003-Support-customize-FL_INDEX_MAX-to-reduce-the-memory-.patch
        && patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0004-Add-tlsf_extend_pool-function.patch &&
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0005-Fix-warnining-on-implicit-pointer-conversion.patch
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(tlsf_fetch)

    if(NOT tlsf_fetch_POPULATED)
      FetchContent_Populate(tlsf_fetch)
    endif()
  endif()

  # ############################################################################
  # Flags
  # ############################################################################

  set(CFLAGS -Dtlsf_printf=if\(0\)printf)

  # ############################################################################
  # Sources
  # ############################################################################

  set(CSRCS mm_tlsf.c ${TLSF_DIR}/tlsf.c)

  # ############################################################################
  # Library Configuration
  # ############################################################################

  target_sources(mm PRIVATE ${CSRCS})
  target_compile_options(mm PRIVATE ${CFLAGS})

endif()
