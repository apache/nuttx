# ##############################################################################
# arch/arm/src/cmake/Toolchain.cmake
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

# Toolchain

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(ARCH_SUBDIR)

if(CONFIG_ARCH_ARMV7A) # ARMv7-A
  set(ARCH_SUBDIR armv7-a)
elseif(CONFIG_ARCH_ARMV7R) # ARMv7-R
  set(ARCH_SUBDIR armv7-r)
elseif(CONFIG_ARCH_ARMV8R) # ARMv8-R
  set(ARCH_SUBDIR armv8-r)
elseif(CONFIG_ARCH_ARMV7M) # ARMv7-M
  set(ARCH_SUBDIR armv7-m)
elseif(CONFIG_ARCH_ARMV8M) # ARMv8-M
  set(ARCH_SUBDIR armv8-m)
elseif(CONFIG_ARCH_ARMV6M) # ARMv6-M
  set(ARCH_SUBDIR armv6-m)
else() # ARM9, ARM7TDMI, etc.
  set(ARCH_SUBDIR arm)
endif()

include(${ARCH_SUBDIR})

# include the toolchain specific cmake file

set(TOOLCHAIN_FILE)

if(CONFIG_ARCH_TOOLCHAIN_CLANG) # clang
  set(TOOLCHAIN_FILE clang)
elseif(CONFIG_ARCH_TOOLCHAIN_ARMCLANG) # arm clang
  set(TOOLCHAIN_FILE armclang)
elseif(CONFIG_ARCH_TOOLCHAIN_GHS) # greenhills
  set(TOOLCHAIN_FILE ghs)
else() # gcc
  set(TOOLCHAIN_FILE gcc)
endif()

include(${TOOLCHAIN_FILE})
