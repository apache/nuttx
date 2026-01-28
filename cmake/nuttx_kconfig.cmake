# ##############################################################################
# cmake/nuttx_kconfig.cmake
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

macro(encode_brackets contents)
  string(REGEX REPLACE "\\[" "__OPEN_BRACKET__" ${contents} "${${contents}}")
  string(REGEX REPLACE "\\]" "__CLOSE_BRACKET__" ${contents} "${${contents}}")
endmacro()

macro(decode_brackets contents)
  string(REGEX REPLACE "__OPEN_BRACKET__" "[" ${contents} "${${contents}}")
  string(REGEX REPLACE "__CLOSE_BRACKET__" "]" ${contents} "${${contents}}")
endmacro()

macro(encode_semicolon contents)
  string(REGEX REPLACE ";" "__SEMICOLON__" ${contents} "${${contents}}")
endmacro()

macro(decode_semicolon contents)
  string(REGEX REPLACE "__SEMICOLON__" ";" ${contents} "${${contents}}")
endmacro()

function(nuttx_export_kconfig_by_value kconfigfile config)
  file(STRINGS ${kconfigfile} ConfigContents)
  encode_brackets(ConfigContents)
  foreach(NameAndValue ${ConfigContents})
    decode_brackets(NameAndValue)
    encode_semicolon(NameAndValue)
    string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})
    string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})
    if(Name STREQUAL ${config})
      string(REPLACE "${Name}=" "" Value ${NameAndValue})
      string(REPLACE "\"" "" Value ${Value})
      decode_semicolon(Value)
      set(${Name}
          ${Value}
          PARENT_SCOPE)
      break()
    endif()
  endforeach()
endfunction()

function(nuttx_export_kconfig kconfigfile)
  # First delete the expired configuration items
  get_property(expired_keys GLOBAL PROPERTY NUTTX_CONFIG_KEYS)
  foreach(key ${expired_keys})
    set(${key} PARENT_SCOPE)
  endforeach()
  file(STRINGS ${kconfigfile} ConfigContents)
  encode_brackets(ConfigContents)
  foreach(NameAndValue ${ConfigContents})
    decode_brackets(NameAndValue)
    encode_semicolon(NameAndValue)
    # Strip leading spaces
    string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})

    # Find variable name
    string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})

    if(Name)
      # Find the value
      string(REPLACE "${Name}=" "" Value ${NameAndValue})

      # remove extra quotes
      if(Value)
        string(REPLACE "\"" "" Value ${Value})
      endif()
      decode_semicolon(Value)
      # Set the variable
      set(${Name}
          ${Value}
          PARENT_SCOPE)
      set_property(GLOBAL APPEND PROPERTY NUTTX_CONFIG_KEYS ${Name})
    endif()
  endforeach()

  # Compatibility for symbols usually user-settable (now, set via env vars)
  # TODO: change usage of these symbols into the corresponding cmake variables
  if(LINUX)
    set(CONFIG_HOST_LINUX
        true
        PARENT_SCOPE)
  elseif(APPLE)
    set(CONFIG_HOST_MACOS
        true
        PARENT_SCOPE)
  elseif(WIN32)
    set(CONFIG_HOST_WINDOWS
        true
        PARENT_SCOPE)
  else()
    set(CONFIG_HOST_OTHER
        true
        PARENT_SCOPE)
  endif()
endfunction()

function(nuttx_generate_kconfig)
  nuttx_parse_function_args(
    FUNC
    nuttx_generate_kconfig
    ONE_VALUE
    MENUDESC
    MULTI_VALUE
    EXTERNAL_DIRECTORIES
    REQUIRED
    ARGN
    ${ARGN})

  # Exit early if: - The current directory does NOT contain a CMakeLists.txt
  # (not an app dir), OR - The output Kconfig already exists in the apps binary
  # directory
  if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt"
     OR EXISTS "${NUTTX_APPS_BINDIR}/Kconfig")
    return()
  endif()

  # Determine output Kconfig file path
  set(KCONFIG_OUTPUT_FILE)
  if(MENUDESC)
    string(REPLACE "/" "_" KCONFIG_PREFIX ${CMAKE_CURRENT_LIST_DIR})
    if(WIN32)
      # On Windows, also replace drive letter separators like "C:".
      string(REPLACE ":" "_" KCONFIG_PREFIX ${KCONFIG_PREFIX})
    endif()

    # Output Kconfig file path: <apps_bindir>/<prefix>_Kconfig
    string(APPEND KCONFIG_OUTPUT_FILE ${NUTTX_APPS_BINDIR} "/"
           ${KCONFIG_PREFIX} "_Kconfig")

    # Start a menu block
    file(WRITE ${KCONFIG_OUTPUT_FILE} "menu \"${MENUDESC}\"\n")
  else()
    # Without MENUDESC, generate the root Kconfig file.
    set(KCONFIG_OUTPUT_FILE ${NUTTX_APPS_BINDIR}/Kconfig)
  endif()

  # Collect valid directories that contain Kconfig or CMakeLists
  set(DIR_LIST)
  foreach(external_dir ${EXTERNAL_DIRECTORIES})
    get_filename_component(external_reldir "${external_dir}" REALPATH)
    if(IS_DIRECTORY ${external_reldir})
      list(APPEND DIR_LIST "${external_reldir}")
    endif()
  endforeach()

  # Get all entries directly under the current app directory
  file(GLOB FIRST_LEVEL_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/*")

  # Add directories that contain either CMakeLists.txt or Kconfig
  foreach(dir IN LISTS FIRST_LEVEL_DIRS)
    if(IS_DIRECTORY "${dir}")
      if(EXISTS "${dir}/CMakeLists.txt" OR EXISTS "${dir}/Kconfig")
        list(APPEND DIR_LIST "${dir}")
      endif()
    endif()
  endforeach()

  # Generate "source" entries for each discovered Kconfig
  foreach(dir ${DIR_LIST})
    set(SUB_KCONFIG "${dir}/Kconfig")
    string(REPLACE "/" "_" MENUCONFIG ${SUB_KCONFIG})
    if(WIN32)
      string(REPLACE ":" "_" MENUCONFIG ${MENUCONFIG})
    endif()

    # Source Kconfig from the corresponding apps binary directory if present
    if(EXISTS ${NUTTX_APPS_BINDIR}/${MENUCONFIG})
      file(APPEND ${KCONFIG_OUTPUT_FILE}
           "source \"${NUTTX_APPS_BINDIR}/${MENUCONFIG}\"\n")
    endif()

    # Source Kconfig from the directory if present
    if(EXISTS ${SUB_KCONFIG})
      file(APPEND ${KCONFIG_OUTPUT_FILE} "source \"${SUB_KCONFIG}\"\n")
    endif()
  endforeach()

  # Close the menu block if MENUDESC was used
  if(MENUDESC)
    file(APPEND ${KCONFIG_OUTPUT_FILE} "endmenu # ${MENUDESC}\n")
  endif()
endfunction()

function(nuttx_olddefconfig)
  execute_process(
    COMMAND olddefconfig
    ERROR_VARIABLE KCONFIG_ERROR
    OUTPUT_VARIABLE KCONFIG_OUTPUT
    RESULT_VARIABLE KCONFIG_STATUS
    WORKING_DIRECTORY ${NUTTX_DIR})

  if(KCONFIG_ERROR)
    message(WARNING "Kconfig Configuration Error: ${KCONFIG_ERROR}")
  endif()

  if(KCONFIG_STATUS AND NOT KCONFIG_STATUS EQUAL 0)
    message(
      FATAL_ERROR
        "nuttx_olddefconfig: Failed to initialize Kconfig configuration: ${KCONFIG_OUTPUT}"
    )
  endif()

  # save the orig compressed formatted defconfig at the very beginning
  execute_process(COMMAND savedefconfig --out ${CMAKE_BINARY_DIR}/defconfig.tmp
                  WORKING_DIRECTORY ${NUTTX_DIR})

  execute_process(
    COMMAND
      ${CMAKE_COMMAND} -P ${NUTTX_DIR}/cmake/savedefconfig.cmake
      ${CMAKE_BINARY_DIR}/.config.compressed ${CMAKE_BINARY_DIR}/defconfig.tmp
      ${CMAKE_BINARY_DIR}/defconfig.orig
    WORKING_DIRECTORY ${NUTTX_DIR})

endfunction()

function(nuttx_setconfig)
  set(ENV{KCONFIG_CONFIG} ${CMAKE_BINARY_DIR}/.config)
  execute_process(
    COMMAND setconfig ${ARGN} --kconfig ${NUTTX_DIR}/Kconfig
    ERROR_VARIABLE KCONFIG_ERROR
    OUTPUT_VARIABLE KCONFIG_OUTPUT
    RESULT_VARIABLE KCONFIG_STATUS
    WORKING_DIRECTORY ${NUTTX_DIR})

  if(KCONFIG_ERROR)
    message(WARNING "Kconfig Configuration Error: ${KCONFIG_ERROR}")
  endif()

  if(KCONFIG_STATUS AND NOT KCONFIG_STATUS EQUAL 0)
    message(
      FATAL_ERROR
        "nuttx_setconfig: Failed to initialize Kconfig configuration: ${KCONFIG_OUTPUT}"
    )
  endif()
endfunction()
