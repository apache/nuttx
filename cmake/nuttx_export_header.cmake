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
# `include_path` directory and preserve their relative relationship
#
# When the target relative path is repeated and the original path is different
# throws an exception and abort the build
function(export_relative_path_recursively include_path export_path
         include_directory)
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
        # create symbolic link for origin
        file(CREATE_LINK ${FOUND_FILE} ${include_path}/${REL_FILE}
             COPY_ON_ERROR SYMBOLIC)
        # do export to the BINAPPS/include directory
        get_filename_component(DES_PATH ${export_path}/${REL_FILE} DIRECTORY)
        install(FILES ${FOUND_FILE} DESTINATION ${DES_PATH})
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
#   foo/include/                            build/apps/include/foo/
#       ├── demo/                                     ├── demo/
#       |   └── demo.h   cmake --build:origin link    |   └── demo.h
#       ├── test/        =====================>       ├── test/
#       |   └── test.h   cmake --install:export       |   └── test.h
#       └── foo.h                                     └── foo.h
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

  # destination directory to be exported by install
  set(EXPORT_PATH_PREFIX ${CMAKE_BINARY_DIR}/staging/include/${TARGET})
  # destination directory origin compile
  set(INCLUDE_PATH_PREFIX ${NUTTX_APPS_BINDIR}/include/${TARGET})

  foreach(INCDIR ${INCLUDE_DIRECTORIES})
    if(NOT EXISTS ${INCDIR})
      message(
        WARNING
          "warning: NUTTX_EXPORT_HEADER ${TARGET} added a path that does not exist ${INCDIR}"
      )
    else()
      if(IS_DIRECTORY ${INCDIR})
        # export relative path to directories
        export_relative_path_recursively(${INCLUDE_PATH_PREFIX}
                                         ${EXPORT_PATH_PREFIX} ${INCDIR})
      else()
        # export single file
        file(CREATE_LINK ${INCDIR} ${INCLUDE_PATH_PREFIX}/${INCDIR}
             COPY_ON_ERROR SYMBOLIC)
        install(FILES ${INCDIR} DESTINATION ${EXPORT_PATH_PREFIX})
      endif()
    endif()
  endforeach()
endfunction()
