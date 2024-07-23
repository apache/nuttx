# ##############################################################################
# cmake/nuttx_kconfig.cmake
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
    file(
      GLOB subdir
      LIST_DIRECTORIES false
      ${NUTTX_APPS_BINDIR} ${NUTTX_APPS_BINDIR}/*_Kconfig)
    foreach(dir ${subdir})
      file(APPEND ${KCONFIG_OUTPUT_FILE} "source \"${dir}\"\n")
    endforeach()
  endif()

  file(
    GLOB subdir
    LIST_DIRECTORIES false
    ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/*/Kconfig)

  foreach(dir ${subdir})
    file(APPEND ${KCONFIG_OUTPUT_FILE} "source \"${dir}\"\n")
  endforeach()

  if(MENUDESC)
    file(APPEND ${KCONFIG_OUTPUT_FILE} "endmenu # ${MENUDESC}\n")
  endif()
endfunction()
