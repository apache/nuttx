# ##############################################################################
# libs/libc/tls/CMakeLists.txt
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

set(SRCS task_getinfo.c tls_getinfo.c)

if(NOT CONFIG_TLS_TASK_NELEM EQUAL 0)
  list(APPEND SRCS task_tls.c task_tls_destruct.c)
endif()

if(NOT CONFIG_TLS_NELEM EQUAL 0)
  list(APPEND SRCS tls_destruct.c)
endif()

if(NOT CONFIG_TLS_NCLEANUP EQUAL 0)
  list(APPEND SRCS tls_cleanup.c)
endif()

# A GCC toolchain configured with --disable-tls compiles a thread_local variable
# into a call to __emutls_get_address(), and the libgcc of such a toolchain
# keeps one copy of the variable for the whole program. Replace it with the
# implementation of tls_emutls.c, which keeps one copy per thread. The symbols
# of libgcc have external linkage and are pulled into the link of some
# configurations whatever the order of the archives is, so redirect the calls
# with --wrap rather than define the same names.
if("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU"
   AND NOT CONFIG_SCHED_THREAD_LOCAL
   AND NOT CONFIG_DISABLE_PTHREAD
   AND NOT CONFIG_TLS_NELEM EQUAL 0
   AND NOT CONFIG_TLS_TASK_NELEM EQUAL 0)
  list(APPEND SRCS tls_emutls.c)
  target_link_options(nuttx PRIVATE -Wl,--wrap=__emutls_get_address
                      -Wl,--wrap=__emutls_register_common)
endif()

target_sources(c PRIVATE ${SRCS})
