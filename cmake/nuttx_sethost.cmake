# ##############################################################################
# cmake/nuttx_sethost.cmake
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
  execute_process(
    COMMAND uname -m
    COMMAND tr -d '\n'
    OUTPUT_VARIABLE ARCHITECTURE)

  if(CONFIG_HOST_WINDOWS)
    message(" Select HOST_WINDOWS=y")
    nuttx_setconfig(HOST_WINDOWS=y)
    nuttx_setconfig(SIM_X8664_MICROSOFT=y)

    # TODO: Enable windows build system: cygwin, mysys, native, etc..
  else()
    if(CONFIG_HOST_LINUX)
      message(" Select HOST_LINUX=y")
      nuttx_setconfig(HOST_LINUX=y)
    elseif(CONFIG_HOST_MACOS)
      message(" Select HOST_MACOS=y")
      nuttx_setconfig(HOST_MACOS=y)
    elseif(CONFIG_HOST_BSD)
      message(" Select HOST_BSD=y")
      nuttx_setconfig(HOST_BSD=y)
    endif()

    # Enable the System V ABI
    nuttx_setconfig(SIM_X8664_SYSTEMV=y)
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
