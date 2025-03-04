# ##############################################################################
# boards/arm/stm32/stm32f411-minimum/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_bringup.c)

if(CONFIG_NSH_LIBRARY)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
endif()

if(CONFIG_USERLED)
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_RGBLED)
  list(APPEND SRCS stm32_rgbled.c)
endif()

if(CONFIG_SPI)
  list(APPEND SRCS stm32_spi.c)
endif()

if(CONFIG_MTD_W25)
  list(APPEND SRCS stm32_w25.c)
endif()

if(CONFIG_STM32_OTGFS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS stm32_composite.c)
endif()

if(CONFIG_STM32F411MINIMUM_GPIO)
  list(APPEND SRCS stm32_gpio.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS stm32_usbmsc.c)
endif()

if(CONFIG_ADC_HX711)
  list(APPEND SRCS stm32_hx711.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT
                             "${NUTTX_BOARD_DIR}/scripts/stm32f411ce.ld")
