# ##############################################################################
# arch/tricore/src/cmake/tc3xx.cmake
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

set(PLATFORM_FLAGS)

if(CONFIG_ARCH_CHIP_TC397)
  if(CONFIG_TRICORE_TOOLCHAIN_TASKING)
    add_compile_options(--cpu=tc39xb)
    add_link_options(-Ctc39xb)
  else()
    list(APPEND PLATFORM_FLAGS -mcpu=tc39xx -mtc162)
  endif()
  list(APPEND PLATFORM_FLAGS
       -I${NUTTX_CHIP_ABS_DIR}/tc397/Libraries/iLLD/TC39B/Tricore/Cpu/Std)
  list(APPEND PLATFORM_FLAGS
       -I${NUTTX_CHIP_ABS_DIR}/tc397/Libraries/Infra/Platform)
  list(APPEND PLATFORM_FLAGS -I${NUTTX_CHIP_ABS_DIR}/tc397/Configurations)
endif()

add_compile_options(${PLATFORM_FLAGS})
add_link_options(${PLATFORM_FLAGS})
