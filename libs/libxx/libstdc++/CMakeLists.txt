# ##############################################################################
# libs/libxx/libstdc++/CMakeLists.txt
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

if(CONFIG_LIBSUPCXX)

  set(LIBSTDCXX_VERSION ${CONFIG_LIBSTDCXX_VERSION})

  # Download and unpack tarball if no git repo found
  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/libstdc++/.git)
    if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/libstdc++)

      FetchContent_Declare(
        libstdcxx
        DOWNLOAD_NAME "libstdc++-v3.tar.gz"
        DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
        URL "https://gnu.googlesource.com/gcc/+archive/refs/tags/releases/gcc-${LIBSTDCXX_VERSION}/libstdc++-v3.tar.gz"
            SOURCE_DIR
            ${CMAKE_CURRENT_LIST_DIR}/libstdc++
            BINARY_DIR
            ${CMAKE_BINARY_DIR}/libs/libxx/libstdc++
            CONFIGURE_COMMAND
            ""
            BUILD_COMMAND
            ""
            INSTALL_COMMAND
            ""
            TEST_COMMAND
            ""
        DOWNLOAD_NO_PROGRESS true
        TIMEOUT 30)

      FetchContent_GetProperties(libstdcxx)

      if(NOT libstdcxx_POPULATED)
        FetchContent_Populate(libstdcxx)
      endif()

    endif()
  endif()

  # Set directories
  set(SUPDIR ${CMAKE_CURRENT_LIST_DIR}/libstdc++/libsupc++)
  set(INCDIR ${CMAKE_CURRENT_LIST_DIR}/libstdc++/include)
  set(CONDIR ${CMAKE_CURRENT_LIST_DIR}/libstdc++/config)

  # Determine CPU config
  if(CONFIG_ARCH_ARM)
    set(CPU_CONFIG arm)
  else()
    set(CPU_CONFIG generic)
  endif()

  # Create include directories
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include/c++)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include/c++/bits)

  # Copy headers to c++ include directory
  set(CXXDIR ${CMAKE_BINARY_DIR}/include/c++)
  set(BITSDIR ${CMAKE_BINARY_DIR}/include/c++/bits)

  # Copy libsupc++ headers
  file(COPY ${CMAKE_CURRENT_LIST_DIR}/c++config.h DESTINATION ${INCDIR}/bits)
  file(COPY ${SUPDIR}/compare DESTINATION ${CXXDIR})
  file(COPY ${SUPDIR}/exception DESTINATION ${CXXDIR})
  file(COPY ${SUPDIR}/initializer_list DESTINATION ${CXXDIR})
  file(COPY ${SUPDIR}/new DESTINATION ${CXXDIR})
  file(COPY ${SUPDIR}/typeinfo DESTINATION ${CXXDIR})

  # Copy gthr headers from libgcc
  file(COPY ${NUTTX_DIR}/libs/libbuiltin/libgcc/libgcc/gthr.h
       DESTINATION ${CXXDIR})
  file(COPY ${NUTTX_DIR}/libs/libbuiltin/libgcc/libgcc/gthr.h
       DESTINATION ${BITSDIR})
  file(COPY_FILE ${NUTTX_DIR}/libs/libbuiltin/libgcc/libgcc/gthr-posix.h
       ${CXXDIR}/gthr-default.h)
  file(COPY_FILE ${NUTTX_DIR}/libs/libbuiltin/libgcc/libgcc/gthr-posix.h
       ${BITSDIR}/gthr-default.h)

  # Copy unwind-pe.h
  file(COPY ${NUTTX_DIR}/libs/libbuiltin/libgcc/libgcc/unwind-pe.h
       DESTINATION ${CMAKE_CURRENT_LIST_DIR})

  # Copy bits headers
  file(COPY ${SUPDIR}/atomic_lockfree_defines.h DESTINATION ${BITSDIR})
  file(COPY ${SUPDIR}/cxxabi_forced.h DESTINATION ${BITSDIR})
  file(COPY ${SUPDIR}/exception_defines.h DESTINATION ${BITSDIR})
  file(COPY ${SUPDIR}/exception_ptr.h DESTINATION ${BITSDIR})
  file(COPY ${SUPDIR}/hash_bytes.h DESTINATION ${BITSDIR})
  file(COPY ${SUPDIR}/nested_exception.h DESTINATION ${BITSDIR})
  file(COPY ${SUPDIR}/exception.h DESTINATION ${BITSDIR})
  file(COPY ${SUPDIR}/cxxabi_init_exception.h DESTINATION ${BITSDIR})
  file(COPY ${CONDIR}/cpu/generic/cpu_defines.h DESTINATION ${BITSDIR})
  file(COPY ${CONDIR}/cpu/${CPU_CONFIG}/cxxabi_tweaks.h DESTINATION ${BITSDIR})
  file(COPY ${CONDIR}/cpu/generic/atomic_word.h DESTINATION ${BITSDIR})
  file(COPY ${CONDIR}/os/generic/os_defines.h DESTINATION ${BITSDIR})

  # Set include directories for nuttx
  set_property(
    TARGET nuttx
    APPEND
    PROPERTY NUTTX_CXX_INCLUDE_DIRECTORIES ${CXXDIR} ${INCDIR} ${SUPDIR}
             ${CMAKE_CURRENT_LIST_DIR})

  # Get source files
  file(GLOB SRCS ${SUPDIR}/*.cc)

  # Remove RTTI sources if RTTI is disabled
  if(NOT CONFIG_CXX_RTTI)
    list(REMOVE_ITEM SRCS ${SUPDIR}/eh_arm.cc ${SUPDIR}/eh_call.cc)
  endif()

  set(FLAGS -Wno-undef -Wno-unused-result)

  # Create library manually to avoid inheriting NUTTX_CXX_INCLUDE_DIRECTORIES
  # which contains libcxx paths that conflict with libstdc++ headers
  add_library(libsupcxx ${SRCS})
  add_dependencies(libsupcxx nuttx_context)

  target_compile_options(libsupcxx PRIVATE ${FLAGS})

  target_include_directories(
    libsupcxx BEFORE PRIVATE ${INCDIR} ${INCDIR}/std ${INCDIR}/c_global
                             ${SUPDIR} ${CMAKE_CURRENT_LIST_DIR} ${CXXDIR})

  # Add system include directories
  target_include_directories(
    libsupcxx SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/include
                             ${CMAKE_BINARY_DIR}/include)

  # Set global compile options & definitions from nuttx target
  target_compile_options(
    libsupcxx
    PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_COMPILE_OPTIONS>>)
  target_compile_definitions(
    libsupcxx PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_DEFINITIONS>>)
  target_include_directories(
    libsupcxx
    PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_INCLUDE_DIRECTORIES>>)

  # Add to system libraries list
  set_property(GLOBAL APPEND PROPERTY NUTTX_SYSTEM_LIBRARIES libsupcxx)

  # Install target
  install(TARGETS libsupcxx)

endif()
