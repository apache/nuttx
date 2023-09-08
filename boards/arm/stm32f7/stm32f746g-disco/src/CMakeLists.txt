# ##############################################################################
# boards/arm/stm32f7/stm32f746-disco/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_spi.c stm32_bringup.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS  stm32_buttons.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinitialize.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_SPORADIC_INSTRUMENTATION)
  list(APPEND SRCS stm32_sporadic.c)
endif()

if(CONFIG_STM32F7_LTDC)
  list(APPEND SRCS stm32_lcd.c)
endif()

if(CONFIG_STM32F7_FMC)
  list(APPEND SRCS stm32_extmem.c)
endif()

if(CONFIG_INPUT_FT5X06)
  list(APPEND SRCS stm32_touchscreen.c)
endif()

if(CONFIG_MTD_N25QXXX)
  list(APPEND SRCS stm32_n25q.c)
endif()

if(CONFIG_STM32F7_OTGFS)
  list(APPEND SRCS stm32_usb.c)
elseif(CONFIG_STM32F7_OTGFSHS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_STM32F7_SDMMC)
  list(APPEND SRCS stm32_sdmmc.c)
endif()

if(CONFIG_AUDIO_WM8994)
  list(APPEND SRCS stm32_wm8994.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
