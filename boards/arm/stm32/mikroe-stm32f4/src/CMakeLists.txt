# ##############################################################################
# boards/arm/stm32/mikroe-stm32f4/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_spi.c)

if(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG)
  list(APPEND SRCS stm32_clockconfig.c)
endif()

if(CONFIG_STM32_OTGFS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_ARCH_CUSTOM_PMINIT)
  list(APPEND SRCS stm32_pm.c)
endif()

if(CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

if(CONFIG_STM32_FSMC)
  list(APPEND SRCS stm32_extmem.c)
endif()

if(CONFIG_INPUT)
  list(APPEND SRCS stm32_touchscreen.c)
endif()

if(CONFIG_LCD_MIO283QT2)
  list(APPEND SRCS stm32_mio283qt2.c)
endif()

if(CONFIG_LCD_MIO283QT9A)
  list(APPEND SRCS stm32_mio283qt9a.c)
endif()

if(CONFIG_AUDIO_VS1053)
  list(APPEND SRCS stm32_vs1053.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
