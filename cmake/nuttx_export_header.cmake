# ##############################################################################
# cmake/nuttx_export_headers.cmake
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

include(nuttx_parse_function_args)

# export_relative_path_recursively
#
# Used to recursively add all header files under the `include_directory` to the
# `prefix` directory and preserve their relative relationship
#
# When the target relative path is repeated and the original path is different
# throws an exception and abort the build
function(export_relative_path_recursively prefix include_directory)
  # recursively find header files under the path
  file(GLOB_RECURSE FOUND_FILES ${include_directory}/*.h)
  if(FOUND_FILES)
    foreach(FOUND_FILE ${FOUND_FILES})
      if(EXISTS ${FOUND_FILE})
        # take relative path
        file(RELATIVE_PATH REL_FILE ${include_directory} ${FOUND_FILE})
        # check for path conflicts
        string(REPLACE "/" "_" CHECK_PROP _check_${prefix}_${REL_FILE})
        get_property(REL_FILE_CHECK GLOBAL PROPERTY ${CHECK_PROP})
        if(REL_FILE_CHECK AND NOT REL_FILE_CHECK STREQUAL FOUND_FILE)
          message(
            FATAL_ERROR
              "Error: NUTTX_ADD_APPS_HEADER add a duplicate header files ${FOUND_FILE} ${REL_FILE_CHECK}"
          )
        endif()
        set_property(GLOBAL PROPERTY ${CHECK_PROP} ${FOUND_FILE})
        message(
          VERBOSE
          "NUTTX_ADD_APPS_HEADER: REL_FILE found: ${REL_FILE} in ${INCDIR}")
        # do export to the BINAPPS/include directory
        get_filename_component(DES_PATH ${prefix}/${REL_FILE} DIRECTORY)
        file(
          COPY ${FOUND_FILE}
          DESTINATION ${DES_PATH}
          FOLLOW_SYMLINK_CHAIN)
      endif()
    endforeach()
  endif()
endfunction()

# ~~~
# nuttx_export_header
#
# Description:
#   Export include directory to apps global include path.
#   `${NUTTX_APPS_BINDIR}/include/${TARGET}`
#   preserve relative paths to all header files.
#
#   In the original GNU Make build, we use `Make.defs` to add the global include path,
#   and the private include path of the application library is written in the `Makefile`
#   In Nuttx CMake build, we achieve similar global functionality through this function.
#
# Usage:
#   nuttx_export_header(TARGET <string> INCLUDE_DIRECTORIES <list>)
#
# Parameters:
#   TARGET                : target for exporting header directory
#   INCLUDE_DIRECTORIES   : directory list to be exported
#
# Example:
#   set(INCDIRS foo/include)
#   nuttx_export_header(TARGET foo INCLUDE_DIRECTORIES ${INCDIRS})
#
#   foo/include/                          build/apps/include/foo/
#       ├── demo/                                   ├── demo/
#       |   └── demo.h        export to             |   └── demo.h
#       ├── test/           ============>           ├── test/
#       |   └── test.h                              |   └── test.h
#       └── foo.h                                   └── foo.h
# ~~~

function(nuttx_export_header)

  # parse arguments into variables

  nuttx_parse_function_args(
    FUNC
    nuttx_export_header
    ONE_VALUE
    TARGET
    MULTI_VALUE
    INCLUDE_DIRECTORIES
    REQUIRED
    TARGET
    INCLUDE_DIRECTORIES
    ARGN
    ${ARGN})

  # destination directory to be exported
  set(PATH_PREFIX ${NUTTX_APPS_BINDIR}/include/${TARGET})

  foreach(INCDIR ${INCLUDE_DIRECTORIES})
    if(NOT EXISTS ${INCDIR})
      message(
        WARNING
          "warning: NUTTX_EXPORT_HEADER ${TARGET} added a path that does not exist ${INCDIR}"
      )
    else()
      if(IS_DIRECTORY ${INCDIR})
        # export relative path to directories
        export_relative_path_recursively(${PATH_PREFIX} ${INCDIR})
      else()
        # export single file
        get_filename_component(DES_PATH ${prefix}/${REL_FILE} DIRECTORY)
        file(
          COPY ${FOUND_FILE}
          DESTINATION ${DES_PATH}
          FOLLOW_SYMLINK_CHAIN)
      endif()
    endif()
  endforeach()
endfunction()
