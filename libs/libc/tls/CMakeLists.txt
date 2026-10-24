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

# Send the calls to __emutls_get_address() to the per-thread version of libc
# (see tools/Config.mk).

if(CONFIG_TLS_EMUTLS)
  list(APPEND SRCS tls_emutls.c)
  set(EMUTLS_WRAP -Wl,--wrap=__emutls_get_address
                  -Wl,--wrap=__emutls_register_common)
  # with LTO the calls appear only after the linker has chosen the archive
  # members: take the libc functions in from the start
  if(CONFIG_LTO_FULL OR CONFIG_LTO_THIN)
    list(APPEND EMUTLS_WRAP -Wl,-u,__wrap___emutls_get_address
         -Wl,-u,__wrap___emutls_register_common)
  endif()
  if(CONFIG_ARCH_SIM)
    # the sim links NuttX into nuttx.rel first (see CMakeLists.txt)
    set_property(GLOBAL PROPERTY NUTTX_SIM_REL_LINK_OPTIONS ${EMUTLS_WRAP})
  else()
    target_link_options(nuttx PRIVATE ${EMUTLS_WRAP})
  endif()
  if(CONFIG_BUILD_PROTECTED)
    target_link_options(nuttx_user PRIVATE ${EMUTLS_WRAP})
  endif()
endif()

if(NOT CONFIG_TLS_TASK_NELEM EQUAL 0)
  list(APPEND SRCS task_tls.c task_tls_destruct.c)
endif()

if(NOT CONFIG_TLS_NELEM EQUAL 0)
  list(APPEND SRCS tls_destruct.c)
endif()

if(NOT CONFIG_TLS_NCLEANUP EQUAL 0)
  list(APPEND SRCS tls_cleanup.c)
endif()

target_sources(c PRIVATE ${SRCS})
