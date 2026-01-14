# ##############################################################################
# libs/libc/spawn/CMakeLists.txt
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

set(SRCS
    lib_psfa_addaction.c
    lib_psfa_addclose.c
    lib_psfa_adddup2.c
    lib_psfa_addopen.c
    lib_psfa_destroy.c
    lib_psfa_init.c
    lib_psa_getflags.c
    lib_psa_getschedparam.c
    lib_psa_getschedpolicy.c
    lib_psa_init.c
    lib_psa_setflags.c
    lib_psa_setschedparam.c
    lib_psa_setschedpolicy.c
    lib_psa_getsigmask.c
    lib_psa_setsigmask.c
    lib_psa_getstacksize.c
    lib_psa_setstacksize.c
    lib_psa_destroy.c)

if(NOT CONFIG_BUILD_KERNEL)
  list(APPEND SRCS lib_psa_getstacksize.c lib_psa_setstacksize.c)
endif()

if(CONFIG_DEBUG_FEATURES)
  list(APPEND SRCS lib_psfa_dump.c lib_psa_dump.c)
endif()

target_sources(c PRIVATE ${SRCS})
