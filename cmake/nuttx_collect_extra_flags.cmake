# ##############################################################################
# cmake/nuttx_collect_extra_flags.cmake
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

# ~~~
# nuttx_collect_extra_flags
#
# Description:
#   Collect the directory-level COMPILE_OPTIONS of the NuttX source tree (e.g.
#   -m32, -march, -mcpu) into NUTTX_EXTRA_FLAGS.  The final link and the steps
#   that invoke the compiler directly (toolchain library lookup,
#   add_custom_command) use these flags so that they match the compiled
#   objects.
#
#   Flags listed in COMPILE_ONLY_FLAGS are dropped, generator expressions of
#   the form $<$<COMPILE_LANGUAGE:C>:...> are unwrapped to their C payload and
#   duplicates are removed.  The result is stored both in the GLOBAL property
#   NUTTX_EXTRA_FLAGS and in the NUTTX_EXTRA_FLAGS variable of the caller.
#
# Usage:
#   nuttx_collect_extra_flags([EXCLUDE <regex> ...])
#
# Parameters:
#   EXCLUDE : optional regular expressions; matching plain flags are skipped
#             (e.g. "-nostdlib").
# ~~~

function(nuttx_collect_extra_flags)
  cmake_parse_arguments(ARG "" "" "EXCLUDE" ${ARGN})

  get_directory_property(TOOLCHAIN_DIR_FLAGS DIRECTORY ${NUTTX_DIR}
                                                       COMPILE_OPTIONS)

  set(extra_flags "")
  foreach(FLAG ${TOOLCHAIN_DIR_FLAGS})
    if(FLAG IN_LIST COMPILE_ONLY_FLAGS)
      continue()
    endif()
    if(NOT FLAG MATCHES "^\\$<.*>$")
      set(skip FALSE)
      foreach(regex ${ARG_EXCLUDE})
        if(FLAG MATCHES "${regex}")
          set(skip TRUE)
          break()
        endif()
      endforeach()
      if(NOT skip)
        list(APPEND extra_flags ${FLAG})
      endif()
    else()
      string(REGEX MATCH "\\$<\\$<COMPILE_LANGUAGE:C>:(.*)>" matched ${FLAG})
      if(matched)
        list(APPEND extra_flags ${CMAKE_MATCH_1})
      endif()
    endif()
  endforeach()

  if(extra_flags)
    list(REMOVE_DUPLICATES extra_flags)
  endif()

  set_property(GLOBAL PROPERTY NUTTX_EXTRA_FLAGS ${extra_flags})
  set(NUTTX_EXTRA_FLAGS
      ${extra_flags}
      PARENT_SCOPE)
endfunction()
