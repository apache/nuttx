# ##############################################################################
# boards/arm/stm32h7/nucleo-h743zi/src/CMakeLists.txt
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

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_STM32_ROMFS)
  list(APPEND SRCS stm32_romfs_initialize.c)
endif()

if(CONFIG_STM32H7_SPI)
  list(APPEND SRCS stm32_spi.c)
endif()

if(CONFIG_STM32H7_OTGFS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS stm32_uid.c)
endif()

if(CONFIG_SENSORS_LSM6DSL)
  list(APPEND SRCS stm32_lsm6dsl.c)
endif()

if(CONFIG_SENSORS_LSM9DS1)
  list(APPEND SRCS stm32_lsm9ds1.c)
endif()

if(CONFIG_SENSORS_LSM303AGR)
  list(APPEND SRCS stm32_lsm303agr.c)
endif()

if(CONFIG_PCA9635PW)
  list(APPEND SRCS stm32_pca9635.c)
endif()

if(CONFIG_LCD_SSD1306)
  list(APPEND SRCS stm32_ssd1306.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinitialize.c)
endif()

if(CONFIG_STM32H7_PROGMEM)
  list(APPEND SRCS stm32_progmem.c)
endif()

if(CONFIG_WL_NRF24L01)
  list(APPEND SRCS stm32_nrf24l01.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS stm32_gpio.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS stm32_reset.c)
endif()

if(CONFIG_BOARDCTL_BOOT_IMAGE)
  list(APPEND SRCS stm32_boot_image.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS stm32_usbmsc.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS stm32_composite.c)
endif()

if(CONFIG_MMCSD)
  list(APPEND SRCS stm32_mmcsd.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_STM32_APP_FORMAT_MCUBOOT)
  if(CONFIG_MCUBOOT_BOOTLOADER)
    set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash-mcuboot-loader.ld")
  else()
    set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash-mcuboot-app.ld")
  endif()
else()
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
endif()

if(NOT CONFIG_BUILD_FLAT)
  add_subdirectory(${NUTTX_BOARD_DIR}/kernel)
  set_property(
    GLOBAL PROPERTY LD_SCRIPT_USER ${NUTTX_BOARD_DIR}/scripts/memory.ld
    ${NUTTX_BOARD_DIR}/scripts/user-space.ld)
endif()
