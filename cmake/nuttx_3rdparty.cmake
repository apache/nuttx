# ##############################################################################
# cmake/nuttx_3rdparty.cmake
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
# nuttx_make_nxtmpdir
#
# Description:
#   Creates a persistent third-party cache directory under nuttx/../nxtmpdir
#
# Parameters:
#   NXTMPDIR_PATH: path to the persistent third-party cache directory
# ~~~

function(nuttx_make_nxtmpdir)
  set(_nxtmpdir "${NUTTX_DIR}/../nxtmpdir")
  set(NXTMPDIR_PATH
      "${_nxtmpdir}"
      PARENT_SCOPE)
  if(NOT EXISTS "${_nxtmpdir}")
    file(MAKE_DIRECTORY "${_nxtmpdir}")
  endif()
endfunction()

# ~~~
# nuttx_remove_nxtmpdir
#
# Description:
#   Remove the third-party cache directory under nuttx/../nxtmpdir
#
# ~~~

function(nuttx_remove_nxtmpdir)
  set(_nxtmpdir "${NUTTX_DIR}/../nxtmpdir")
  if(EXISTS "${_nxtmpdir}")
    file(REMOVE_RECURSE "${_nxtmpdir}")
  endif()
endfunction()

# ~~~
# nuttx_check_git_hash
#
# Description:
#   Checks if a given directory is a git repository and if the current
#   commit hash matches the expected one.
#
# Parameters:
#   DIR: path to the directory to check
#   EXPECTED_HASH: the expected commit hash (e.g. "a85ce2f1bad9f745090146eb30a18d91b8ddd309")
#   RESULT_VAR: the variable to store the result (TRUE or FALSE)
# ~~~

function(nuttx_check_git_hash DIR EXPECTED_HASH RESULT_VAR)
  if(NOT IS_DIRECTORY "${DIR}")
    set(${RESULT_VAR}
        FALSE
        PARENT_SCOPE)
    return()
  endif()

  message(STATUS "COMMIT SHA-1: ${EXPECTED_HASH}")
  execute_process(
    COMMAND git -C "${DIR}" rev-parse HEAD
    OUTPUT_VARIABLE CURRENT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET
    RESULT_VARIABLE GIT_RETVAL)
  if(GIT_RETVAL EQUAL 0 AND CURRENT_HASH STREQUAL EXPECTED_HASH)
    set(${RESULT_VAR}
        TRUE
        PARENT_SCOPE)
  else()
    message(
      WARNING
        "Commit hash mismatch in ${DIR} (expected: ${EXPECTED_HASH}, current: ${CURRENT_HASH})"
    )
    set(${RESULT_VAR}
        FALSE
        PARENT_SCOPE)
  endif()
endfunction()
