# ##############################################################################
# libs/libc/machine/arm/CMakeLists.txt
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

set(SRCS)

if(CONFIG_ARCH_ARM7TDMI) # ARM7TDMI is ARMv4T
  add_subdirectory(arm)
elseif(CONFIG_ARCH_ARM920T) # ARM920T is ARMv4T
  add_subdirectory(arm)
elseif(CONFIG_ARCH_ARM926EJS) # ARM926EJS is ARMv5TEJ
  add_subdirectory(arm)
elseif(CONFIG_ARCH_ARM1136J) # ARM1136J is ARMv6
  add_subdirectory(arm)
elseif(CONFIG_ARCH_ARM1156T2) # ARM1156T2 is ARMv6T2
  add_subdirectory(arm)
elseif(CONFIG_ARCH_ARM1176JZ) # ARM1176JZ is ARMv6Z
  add_subdirectory(arm)
elseif(CONFIG_ARCH_ARMV7A) # All ARMv7-A
  add_subdirectory(armv7-a)
elseif(CONFIG_ARCH_ARMV7R) # All ARMv7-R
  add_subdirectory(armv7-r)
elseif(CONFIG_ARCH_ARMV6M) # All ARMv6-M
  add_subdirectory(armv6-m)
elseif(CONFIG_ARCH_ARMV7M) # All ARMv7-M
  add_subdirectory(armv7-m)
elseif(CONFIG_ARCH_ARMV8M) # All ARMv8-M
  add_subdirectory(armv8-m)
endif()

if(NOT CONFIG_LIBSUPCXX)
  list(APPEND SRCS aeabi_atexit.c)
endif()

if(CONFIG_CXX_EXCEPTION)
  list(APPEND SRCS gnu_unwind_find_exidx.c)
endif()

if(CONFIG_ARCH_SETJMP_H)
  if(CONFIG_ARCH_CHIP_TLSR82)
    list(APPEND SRCS gnu/tc32_setjmp.S)
  elseif(CONFIG_ARCH_TOOLCHAIN_GNU)
    list(APPEND SRCS gnu/arch_setjmp.S)
  endif()
endif()

target_sources(c PRIVATE ${SRCS})
