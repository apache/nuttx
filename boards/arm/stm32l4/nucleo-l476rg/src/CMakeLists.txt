# ##############################################################################
# boards/arm/stm32l4/nucleo-l476rg/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_spi.c stm32_bringup.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_WL_CC1101)
  list(APPEND SRCS stm32_cc1101.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
  if(CONFIG_INPUT_AJOYSTICK)
    list(APPEND SRCS stm32_ajoystick.c)
  endif()
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS stm32_gpio.c)
endif()

if(CONFIG_CAN)
  list(APPEND SRCS stm32_can.c)
endif()

if(CONFIG_MMCSD_SPI)
  list(APPEND SRCS stm32_spimmcsd.c)
endif()

if(CONFIG_LCD_PCD8544)
  list(APPEND SRCS stm32_pcd8544.c)
endif()

if(CONFIG_SENSORS_QENCODER)
  list(APPEND SRCS stm32_qencoder.c)
endif()

if(CONFIG_SENSORS_HTS221)
  list(APPEND SRCS stm32_lsm6dsl.c)
endif()

if(CONFIG_SENSORS_LSM303AGR)
  list(APPEND SRCS stm32_lsm303agr.c)
endif()

if(CONFIG_SENSORS_AS726X)
  list(APPEND SRCS stm32_as726x.c)
endif()

if(CONFIG_SENSORS_BMP180)
  list(APPEND SRCS stm32_bmp180.c)
endif()

if(CONFIG_SENSORS_BMP280)
  list(APPEND SRCS stm32_bmp280.c)
endif()

if(CONFIG_SENSORS_MPU9250)
  list(APPEND SRCS stm32_mpu9250.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS stm32_timer.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
  if(CONFIG_BOARDCTL_IOCTL)
    list(APPEND SRCS stm32_ioctl.c)
  endif()
endif()

if(CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS stm32_uid.c)
endif()

if(NOT CONFIG_STM32_ETHMAC)
  if(CONFIG_NETDEVICES)
    list(APPEND SRCS stm32_netinit.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/l476rg.ld")
