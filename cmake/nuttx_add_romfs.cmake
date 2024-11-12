# ##############################################################################
# cmake/nuttx_add_romfs.cmake
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

function(add_board_rcsrcs)
  set_property(
    TARGET board
    APPEND
    PROPERTY BOARD_RCSRCS ${ARGN})
endfunction()

function(add_board_rcraws)
  set_property(
    TARGET board
    APPEND
    PROPERTY BOARD_RCRAWS ${ARGN})
endfunction()

# ~~~
# nuttx_add_romfs
#
# Description:
#   Generates a ROMFS image in a C array, which is built to an OBJECT library.
#
# Parameters:
#   NAME    : determines the romfs label and name of target (romfs_${NAME})
#   HEADER  : option to indicate that a .h file is to be generated instead of a .c
#   PREFIX  : optional prefix to add to image name (as romfs_${PREFIX}.img)
#   NONCONST: option to indicate the array should be non-const
#   DEPENDS : list of targets that should be depended on
# ~~~

function(nuttx_add_romfs)
  nuttx_parse_function_args(
    FUNC
    nuttx_add_romfs
    ONE_VALUE
    NAME
    MOUNTPOINT
    PATH
    PREFIX
    OPTIONS
    HEADER
    NONCONST
    MULTI_VALUE
    DEPENDS
    RCSRCS
    RCRAWS
    REQUIRED
    NAME
    ARGN
    ${ARGN})

  if(NOT PATH AND NOT FILES)
    message(FATAL_ERROR "Either PATH or FILES must be specified")
  endif()

  if(TARGET board)
    get_property(
      board_rcsrcs
      TARGET board
      PROPERTY BOARD_RCSRCS)
    get_property(
      board_rcraws
      TARGET board
      PROPERTY BOARD_RCRAWS)
    list(PREPEND RCSRCS ${board_rcsrcs})
    list(PREPEND RCRAWS ${board_rcraws})
  endif()

  foreach(rcsrc ${RCSRCS})
    if(IS_ABSOLUTE ${rcsrc})
      string(REGEX REPLACE "^(.*)/etc(/.*)?$" "\\1" SOURCE_ETC_PREFIX
                           "${rcsrc}")
      string(REGEX REPLACE "^.*/(etc(/.*)?)$" "\\1" REMAINING_PATH "${rcsrc}")
      string(REGEX REPLACE "^/" "" SOURCE_ETC_SUFFIX "${REMAINING_PATH}")
    else()
      set(SOURCE_ETC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
      set(SOURCE_ETC_SUFFIX ${rcsrc})
    endif()

    get_filename_component(rcpath ${SOURCE_ETC_SUFFIX} DIRECTORY)
    if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${rcpath})
      file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${rcpath})
    endif()
    nuttx_generate_preprocess_target(
      SOURCE_FILE ${SOURCE_ETC_PREFIX}/${SOURCE_ETC_SUFFIX} TARGET_FILE
      ${CMAKE_CURRENT_BINARY_DIR}/${SOURCE_ETC_SUFFIX} DEPENDS nuttx_context)
    list(APPEND DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${SOURCE_ETC_SUFFIX})
  endforeach()

  foreach(rcraw ${RCRAWS})

    if(IS_ABSOLUTE ${rcraw})
      string(REGEX REPLACE "^(.*)/etc(/.*)?$" "\\1" SOURCE_ETC_PREFIX
                           "${rcraw}")
      string(REGEX REPLACE "^.*/(etc(/.*)?)$" "\\1" REMAINING_PATH "${rcraw}")
      string(REGEX REPLACE "^/" "" SOURCE_ETC_SUFFIX "${REMAINING_PATH}")
    else()
      set(SOURCE_ETC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
      set(SOURCE_ETC_SUFFIX ${rcraw})
    endif()

    if(IS_DIRECTORY ${SOURCE_ETC_PREFIX}/${SOURCE_ETC_SUFFIX})
      add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${SOURCE_ETC_SUFFIX}
        COMMAND ${CMAKE_COMMAND} -E make_directory
                ${CMAKE_CURRENT_BINARY_DIR}/${SOURCE_ETC_SUFFIX}
        COMMAND
          ${CMAKE_COMMAND} -E copy_directory
          ${SOURCE_ETC_PREFIX}/${SOURCE_ETC_SUFFIX}
          ${CMAKE_CURRENT_BINARY_DIR}/${SOURCE_ETC_SUFFIX})
      list(APPEND DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${SOURCE_ETC_SUFFIX})
    else()
      list(APPEND DEPENDS ${SOURCE_ETC_PREFIX}/${SOURCE_ETC_SUFFIX})
      configure_file(${SOURCE_ETC_PREFIX}/${SOURCE_ETC_SUFFIX}
                     ${CMAKE_CURRENT_BINARY_DIR}/${SOURCE_ETC_SUFFIX} COPYONLY)
    endif()
  endforeach()

  if(HEADER)
    set(EXTENSION h)
  else()
    set(EXTENSION c)
  endif()

  if(PREFIX)
    set(IMGNAME ${PREFIX}.img)
  else()
    set(IMGNAME romfs.img)
  endif()

  add_custom_command(
    OUTPUT romfs_${NAME}.${EXTENSION}
    COMMAND ${CMAKE_COMMAND} -E make_directory romfs_${NAME}
    COMMAND if \[ \"${PATH}\" != \"\" \]; then ${CMAKE_COMMAND} -E
            copy_directory ${PATH} romfs_${NAME} \; fi
    COMMAND genromfs -f ${IMGNAME} -d romfs_${NAME} -V ${NAME}
    COMMAND xxd -i ${IMGNAME} romfs_${NAME}.${EXTENSION}
    COMMAND if ! [ -z "${NONCONST}" ]\; then sed -E -i'' -e
            "s/^unsigned/const unsigned/g" romfs_${NAME}.${EXTENSION} \; fi
    DEPENDS ${DEPENDS})

  if(NOT HEADER)
    add_custom_target(target-romfs DEPENDS ${DEPENDS})
    nuttx_add_aux_library(romfs_${NAME})
    target_sources(romfs_${NAME} PRIVATE romfs_${NAME}.${EXTENSION})
    add_dependencies(romfs_${NAME} target-romfs)
  endif()
endfunction()

# nuttx_add_cromfs Generates a CROMFS image in a C array, which is built to an
# OBJECT library.
#
# Parameters: - NAME: determines the name of target (cromfs_${NAME}) - PATH: the
# directory that will be used to create the CROMFS - FILES: paths to files to
# copy into CROMFS - DEPENDS: list of targets that should be depended on

function(nuttx_add_cromfs)
  nuttx_parse_function_args(
    FUNC
    nuttx_add_cromfs
    ONE_VALUE
    NAME
    MOUNTPOINT
    PATH
    MULTI_VALUE
    DEPENDS
    FILES
    OPTIONS
    REQUIRED
    NAME
    ARGN
    ${ARGN})

  if(NOT PATH AND NOT FILES)
    message(FATAL_ERROR "Either PATH or FILES must be specified")
  endif()

  add_custom_command(
    OUTPUT cromfs_${NAME}.c
    COMMAND ${CMAKE_COMMAND} -E make_directory cromfs_${NAME}
    COMMAND if \[ \"${PATH}\" != \"\" \]; then ${CMAKE_COMMAND} -E
            copy_directory ${PATH} cromfs_${NAME} \; fi
    COMMAND if \[ \"${FILES}\" != \"\" \]; then ${CMAKE_COMMAND} -E copy
            ${FILES} cromfs_${NAME} \; fi
    COMMAND ${CMAKE_BINARY_DIR}/bin/gencromfs cromfs_${NAME} cromfs_${NAME}.c
    DEPENDS ${DEPENDS})

  add_library(cromfs_${NAME} OBJECT cromfs_${NAME}.c)
  nuttx_add_library_internal(cromfs_${NAME})
endfunction()
