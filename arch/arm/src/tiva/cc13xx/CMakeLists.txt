# ##############################################################################
# arch/arm/src/tiva/cc13xx/CMakeLists.txt
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

if((CONFIG_ARCH_CHIP_CC13X0) OR (CONFIG_ARCH_CHIP_CC13X2))
  set(SRCS)

  if(CONFIG_ARCH_CHIP_CC13X0)
    list(
      APPEND
      SRCS
      cc13xx_start.c
      cc13xx_prcm.c
      cc13xx_chipinfo.c
      cc13xx_gpio.c
      cc13xx_gpioirq.c
      cc13xx_enableclks.c
      cc13xx_enablepwr.c
      cc13x0_trim.c
      cc13x0_rom.c)
  elseif(CONFIG_ARCH_CHIP_CC13X2)
    list(
      APPEND
      SRCS
      cc13xx_start.c
      cc13xx_prcm.c
      cc13xx_chipinfo.c
      cc13xx_gpio.c
      cc13xx_gpioirq.c
      cc13xx_enableclks.c
      cc13xx_enablepwr.c
      cc13x2_aux_sysif.c)
    if(CONFIG_ARCH_CHIP_CC13XX_V1)
      list(APPEND SRCS cc13x2_v1_trim.c cc13x2_cc26x2_v1_rom.c)
    else()
      list(APPEND SRCS cc13x2_v2_trim.c)
    endif()
  endif()

  target_sources(arch PRIVATE ${SRCS})

endif()
