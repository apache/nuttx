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
      string(REPLACE "\"" "" Value ${Value})
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
    REQUIRED
    ARGN
    ${ARGN})
  if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt"
     OR EXISTS "${NUTTX_APPS_BINDIR}/Kconfig")
    return()
  endif()

  set(KCONFIG_OUTPUT_FILE)
  if(MENUDESC)
    string(REPLACE "/" "_" KCONFIG_PREFIX ${CMAKE_CURRENT_LIST_DIR})
    if(WIN32)
      string(REPLACE ":" "_" KCONFIG_PREFIX ${KCONFIG_PREFIX})
    endif()
    string(APPEND KCONFIG_OUTPUT_FILE ${NUTTX_APPS_BINDIR} "/"
           ${KCONFIG_PREFIX} "_Kconfig")
    file(WRITE ${KCONFIG_OUTPUT_FILE} "menu \"${MENUDESC}\"\n")
  else()
    set(KCONFIG_OUTPUT_FILE ${NUTTX_APPS_BINDIR}/Kconfig)
  endif()

  file(
    GLOB SUB_CMAKESCRIPTS
    LIST_DIRECTORIES false
    ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/*/CMakeLists.txt)

  # we need to recursively generate the Kconfig menus of multi-level
  # directories.
  #
  # when generating a Kconfig file for the current directory, it should include
  # and invoke all the Kconfig files gathered from its subdirectories.
  foreach(SUB_CMAKESCRIPT ${SUB_CMAKESCRIPTS})
    string(REPLACE "CMakeLists.txt" "Kconfig" SUB_KCONFIG ${SUB_CMAKESCRIPT})
    string(REPLACE "/" "_" MENUCONFIG ${SUB_KCONFIG})
    if(WIN32)
      string(REPLACE ":" "_" MENUCONFIG ${MENUCONFIG})
    endif()
    # check whether the subdirectory will include a generated Kconfig file.
    if(EXISTS ${NUTTX_APPS_BINDIR}/${MENUCONFIG})
      file(APPEND ${KCONFIG_OUTPUT_FILE}
           "source \"${NUTTX_APPS_BINDIR}/${MENUCONFIG}\"\n")
    elseif(EXISTS ${SUB_KCONFIG})
      file(APPEND ${KCONFIG_OUTPUT_FILE} "source \"${SUB_KCONFIG}\"\n")
    endif()
  endforeach()

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
