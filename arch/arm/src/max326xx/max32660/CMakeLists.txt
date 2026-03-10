# ##############################################################################
# arch/arm/src/max326xx/max32660/CMakeLists.txt
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

if(CONFIG_ARCH_FAMILY_MAX32660)
  set(SRCS)

  if(CONFIG_MAX326XX_ICC)
    list(APPEND SRCS max32660_icc.c)
  endif()

  list(APPEND SRCS max32660_clockconfig.c max32660_lowputc.c max32660_gpio.c)

  if(CONFIG_MAX326XX_DMA)
    list(APPEND SRCS max32660_dma.c)
  endif()

  if(CONFIG_MAX326XX_GPIOIRQ)
    list(APPEND SRCS max32660_gpioirq.c)
  endif()

  if(CONFIG_MAX326XX_RTC)
    list(APPEND SRCS max32660_rtc.c)
  endif()

  if(CONFIG_MAX326XX_WDOG)
    list(APPEND SRCS max32660_wdt.c)
  endif()

  if(CONFIG_MAX326XX_HAVE_UART)
    list(APPEND SRCS max32660_serial.c)
  endif()

  if(CONFIG_MAX326XX_HAVE_I2CM)
    list(APPEND SRCS max32660_i2cm.c)
  endif()

  if(CONFIG_MAX326XX_HAVE_SPIM)
    if(CONFIG_MAX326XX_SPIM0)
      list(APPEND SRCS max32660_spim.c)
    endif()
    if(CONFIG_MAX326XX_SPIM1)
      list(APPEND SRCS max32660_spimssm.c)
    endif()
  endif()

  target_sources(arch PRIVATE ${SRCS})

endif()
