# ##############################################################################
# arch/arm/src/tiva/lm/CMakeLists.txt
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

if(CONFIG_ARCH_CHIP_LM)
  set(SRCS)

  if(CONFIG_ARCH_CHIP_LM3S)
    list(APPEND SRCS lm3s_gpio.c)
  elseif(CONFIG_ARCH_CHIP_LM4F)
    list(APPEND SRCS lm4f_gpio.c)
  endif()

  if((CONFIG_TIVA_ETHERNET) AND (CONFIG_ARCH_CHIP_LM3S))
    list(APPEND SRCS lm3s_ethernet.c)
  endif()

  target_sources(arch PRIVATE ${SRCS})

endif()
