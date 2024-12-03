# ##############################################################################
# arch/arm/src/cmake/armv7-m.cmake
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

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

set(TOOLCHAIN_ARCH_FILE)

if(CONFIG_ARCH_TOOLCHAIN_CLANG) # clang
  set(TOOLCHAIN_ARCH_FILE armv7-m_clang)
elseif(CONFIG_ARCH_TOOLCHAIN_ARMCLANG) # arm clang
  set(TOOLCHAIN_ARCH_FILE armv7-m_armclang)
elseif(CONFIG_ARCH_TOOLCHAIN_GHS) # greenhills
  set(TOOLCHAIN_ARCH_FILE armv7-m_ghs)
else() # gcc
  set(TOOLCHAIN_ARCH_FILE armv7-m_gcc)
endif()

include(${TOOLCHAIN_ARCH_FILE})
