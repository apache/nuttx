# ##############################################################################
# libs/libc/stdlib/CMakeLists.txt
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

# Add the stdlib C files to the build

set(SRCS
    lib_abs.c
    lib_abort.c
    lib_atof.c
    lib_atoi.c
    lib_getprogname.c
    lib_atol.c
    lib_atoll.c
    lib_div.c
    lib_ldiv.c
    lib_lldiv.c
    lib_exit.c
    lib_itoa.c
    lib_labs.c
    lib_llabs.c
    lib_realpath.c
    lib_bsearch.c
    lib_rand.c
    lib_rand48.c
    lib_reallocarray.c
    lib_qsort.c
    lib_srand.c
    lib_strtol.c
    lib_strtoll.c
    lib_strtoul.c
    lib_strtoull.c
    lib_strtold.c
    lib_checkbase.c
    lib_mktemp.c
    lib_mkstemp.c
    lib_mkdtemp.c
    lib_aligned_alloc.c
    lib_posix_memalign.c
    lib_valloc.c
    lib_mblen.c
    lib_mbtowc.c
    lib_wctomb.c
    lib_mbstowcs.c
    lib_wcstombs.c
    lib_arc4random.c
    lib_atexit.c)

if(CONFIG_PSEUDOTERM)
  list(APPEND SRCS lib_ptsname.c lib_ptsnamer.c lib_unlockpt.c lib_openpty.c)
endif()

target_sources(c PRIVATE ${SRCS})
