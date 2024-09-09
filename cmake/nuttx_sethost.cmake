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
        STATUS "ENV{PROCESSOR_ARCHITEW6432} = $ENV{PROCESSOR_ARCHITEW6432}")
    else()
      set(CMAKE_HOST_SYSTEM_PROCESSOR "$ENV{PROCESSOR_ARCHITECTURE}")
      message(
        STATUS "ENV{PROCESSOR_ARCHITECTURE} = $ENV{PROCESSOR_ARCHITECTURE}")
    endif()
  else()
    execute_process(
      COMMAND uname -m
      COMMAND tr -d '\n'
      OUTPUT_VARIABLE ARCHITECTURE)
  endif()

  if(CMAKE_HOST_SYSTEM_NAME MATCHES "Linux|Darwin|FreeBSD")
    nuttx_setconfig(HOST_WINDOWS=n)
    if(CMAKE_HOST_SYSTEM_NAME MATCHES "Linux")
      message(" Select HOST_LINUX=y")
      nuttx_setconfig(HOST_LINUX=y)
    elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
      message(" Select HOST_MACOS=y")
      nuttx_setconfig(HOST_MACOS=y)
    elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "FreeBSD")
      message(" Select HOST_BSD=y")
      nuttx_setconfig(HOST_BSD=y)
    endif()
    # Enable the System V ABI
    nuttx_setconfig(SIM_X8664_SYSTEMV=y)
  elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "MSYS|CYGWIN|Windows")
    # Enable Windows and the Microsoft ABI
    message(" Select HOST_WINDOWS=y")
    nuttx_setconfig(HOST_LINUX=n)
    nuttx_setconfig(HOST_MACOS=n)
    nuttx_setconfig(HOST_BSD=n)
    nuttx_setconfig(HOST_WINDOWS=y)
    nuttx_setconfig(SIM_X8664_MICROSOFT=y)
    if(CMAKE_HOST_SYSTEM_NAME MATCHES "CYGWIN")
      message(" Select WINDOWS_CYGWIN=y")
      nuttx_setconfig(WINDOWS_CYGWIN=y)
    elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "MSYS")
      message(" Select WINDOWS_MSYS=y")
      nuttx_setconfig(WINDOWS_MSYS=y)
    elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
      message(" Select WINDOWS_NATIVE=y")
      nuttx_setconfig(EXPERIMENTAL=y)
      nuttx_setconfig(WINDOWS_NATIVE=y)
    endif()
  else()
    message(" Select HOST_OTHER=y")
    nuttx_setconfig(HOST_LINUX=n)
    nuttx_setconfig(HOST_MACOS=n)
    nuttx_setconfig(HOST_BSD=n)
    nuttx_setconfig(HOST_WINDOWS=n)
    nuttx_setconfig(HOST_OTHER=y)
    nuttx_setconfig(OTHER_OS=y)
  endif()

  if(ARCHITECTURE STREQUAL "x86_64")
    message(" Select HOST_X86_64=y")
    nuttx_setconfig(HOST_X86_64=y)
  elseif(ARCHITECTURE STREQUAL "x86")
    message(" Select HOST_X86=y")
    nuttx_setconfig(HOST_X86=y)
  elseif(ARCHITECTURE STREQUAL "arm")
    message(" Select HOST_ARM=y")
    nuttx_setconfig(HOST_ARM=y)
  elseif(ARCHITECTURE STREQUAL "arm64")
    message(" Select HOST_ARM64=y")
    nuttx_setconfig(HOST_ARM64=y)
  endif()

endfunction()
