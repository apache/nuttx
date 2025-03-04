# ##############################################################################
# cmake/nuttx_sethost.cmake
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

include(nuttx_kconfig)

function(nuttx_sethost)

  if(CMAKE_HOST_WIN32)
    # https://learn.microsoft.com/en-us/windows/win32/winprog64/wow64-implementation-details
    if(DEFINED ENV{PROCESSOR_ARCHITEW6432})
      set(CMAKE_HOST_SYSTEM_PROCESSOR "$ENV{PROCESSOR_ARCHITEW6432}")
      message(
        STATUS "  ENV{PROCESSOR_ARCHITEW6432} = $ENV{PROCESSOR_ARCHITEW6432}")
    else()
      set(CMAKE_HOST_SYSTEM_PROCESSOR "$ENV{PROCESSOR_ARCHITECTURE}")
      message(
        STATUS "  ENV{PROCESSOR_ARCHITECTURE} = $ENV{PROCESSOR_ARCHITECTURE}")
    endif()
  else()
    execute_process(
      COMMAND uname -m
      COMMAND tr -d '\n'
      OUTPUT_VARIABLE ARCHITECTURE)
  endif()

  set(NUTTX_SYSTEM_SETHOST)

  if(CMAKE_HOST_SYSTEM_NAME MATCHES "Linux|Darwin|FreeBSD")
    if(CMAKE_HOST_SYSTEM_NAME MATCHES "Linux")
      message("  Select HOST_LINUX=y")
      list(APPEND NUTTX_SYSTEM_SETHOST "HOST_LINUX=y")
    elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
      message("  Select HOST_MACOS=y")
      list(APPEND NUTTX_SYSTEM_SETHOST "HOST_MACOS=y")
    elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "FreeBSD")
      message("  Select HOST_BSD=y")
      list(APPEND NUTTX_SYSTEM_SETHOST "HOST_BSD=y")
    endif()
  elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "MSYS|CYGWIN|Windows")
    message("  Select HOST_WINDOWS=y")
    list(APPEND NUTTX_SYSTEM_SETHOST "HOST_WINDOWS=y")
    if(CMAKE_HOST_SYSTEM_NAME MATCHES "CYGWIN")
      message("  Select WINDOWS_CYGWIN=y")
      list(APPEND NUTTX_SYSTEM_SETHOST "WINDOWS_CYGWIN=y")
    elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "MSYS")
      message("  Select WINDOWS_MSYS=y")
      list(APPEND NUTTX_SYSTEM_SETHOST "WINDOWS_MSYS=y")
    elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
      message("  Select WINDOWS_NATIVE=y")
      if(NOT MSVC)
        list(APPEND NUTTX_SYSTEM_SETHOST "EXPERIMENTAL=y")
        list(APPEND NUTTX_SYSTEM_SETHOST "WINDOWS_NATIVE=y")
      else()
        message("  MSVC toolchain")
        list(APPEND NUTTX_SYSTEM_SETHOST "WINDOWS_NATIVE=y")
      endif()
    endif()
  else()
    message("  Select HOST_OTHER=y")
    list(APPEND NUTTX_SYSTEM_SETHOST "HOST_OTHER=y")
    # nuttx_setconfig(OTHER_OS=y)
  endif()

  if("${NUTTX_BOARD}" STREQUAL "sim")
    if(ARCHITECTURE STREQUAL "x86_64")
      message("  Select HOST_X86_64=y")
      list(APPEND NUTTX_SYSTEM_SETHOST "HOST_X86_64=y")
      if(CMAKE_HOST_SYSTEM_NAME MATCHES "Linux|Darwin|FreeBSD")
        # Enable the System V ABI
        list(APPEND NUTTX_SYSTEM_SETHOST "SIM_X8664_SYSTEMV=y")
      elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "MSYS|CYGWIN|Windows")
        # Enable Microsoft ABI and the System V ABI
        list(APPEND NUTTX_SYSTEM_SETHOST "SIM_X8664_SYSTEMV=y")
        list(APPEND NUTTX_SYSTEM_SETHOST "SIM_X8664_MICROSOFT=y")
      endif()
    elseif(ARCHITECTURE STREQUAL "x86")
      message("  Select HOST_X86=y")
      list(APPEND NUTTX_SYSTEM_SETHOST "HOST_X86=y")
    elseif(ARCHITECTURE STREQUAL "arm")
      message("  Select HOST_ARM=y")
      list(APPEND NUTTX_SYSTEM_SETHOST "HOST_ARM=y")
    elseif(ARCHITECTURE STREQUAL "arm64")
      message("  Select HOST_ARM64=y")
      list(APPEND NUTTX_SYSTEM_SETHOST "HOST_ARM64=y")
    endif()
  endif()
  # message("  nuttx_setconfig: ${NUTTX_SYSTEM_SETHOST}")
  nuttx_setconfig("${NUTTX_SYSTEM_SETHOST}")
endfunction()
