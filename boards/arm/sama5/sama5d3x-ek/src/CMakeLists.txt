# ##############################################################################
# boards/arm/sama5/sama5d3x-ek/src/CMakeLists.txt
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

set(SRCS sam_boot.c)

if(CONFIG_SAMA5_SPI0)
  list(APPEND SRCS sam_spi.c)
else()
  if(CONFIG_SAMA5_SPI1)
    list(APPEND SRCS sam_spi.c)
  endif()
endif()

if(CONFIG_SAMA5_DDRCS)
  list(APPEND SRCS sam_sdram.c)
endif()

if(CONFIG_SAMA5_EBICS0_NOR)
  list(APPEND SRCS sam_norflash.c)
endif()

if(CONFIG_SAMA5_EBICS3_NAND)
  list(APPEND SRCS sam_nandflash.c)
endif()

if(CONFIG_SAMA5D3XEK_NOR_MAIN)
  list(APPEND SRCS nor_main.c)
endif()

if(CONFIG_MTD_AT25)
  if(CONFIG_SAMA5_SPI0)
    list(APPEND SRCS sam_at25.c)
  endif()
endif()

if(CONFIG_MTD_AT24XX)
  if(CONFIG_SAMA5_TWI0)
    list(APPEND SRCS sam_at24.c)
  endif()
endif()

if(CONFIG_SAMA5_HSMCI0)
  list(APPEND SRCS sam_hsmci.c)
else()
  if(CONFIG_SAMA5_HSMCI1)
    list(APPEND SRCS sam_hsmci.c)
  endif()
endif()

if(CONFIG_SAMA5_UHPHS)
  list(APPEND SRCS sam_usb.c)
else()
  if(CONFIG_SAMA5_UDPHS)
    list(APPEND SRCS sam_usb.c)
  endif()
endif()

if(CONFIG_SAMA5_EMACA)
  list(APPEND SRCS sam_ethernet.c)
else()
  if(CONFIG_SAMA5_GMAC)
    list(APPEND SRCS sam_ethernet.c)
  endif()
endif()

if(CONFIG_AUDIO_WM8904)
  if(CONFIG_SAMA5_TWI0)
    if(CONFIG_SAMA5_SSC0)
      list(APPEND SRCS sam_wm8904.c)
    endif()
  endif()
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS sam_appinit.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS sam_adc.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS sam_pwm.c)
endif()

if(CONFIG_CAN)
  list(APPEND SRCS sam_can.c)
endif()

if(CONFIG_AUDIO_I2SCHAR)
  if(CONFIG_SAMA5_SSC0)
    list(APPEND SRCS sam_i2schar.c)
  else()
    if(CONFIG_SAMA5_SSC1)
      list(APPEND SRCS sam_i2schar.c)
    endif()
  endif()
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS sam_usbmsc.c)
endif()

if(CONFIG_SAMA5_TSD)
  list(APPEND SRCS sam_touchscreen.c)
endif()

if(CONFIG_VIDEO_OV2640)
  list(APPEND SRCS sam_ov2640.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS sam_autoleds.c)
else()
  list(APPEND SRCS sam_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS sam_buttons.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_SAMA5_BOOT_ISRAM)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/isram.ld")
endif()

if(CONFIG_SAMA5_BOOT_SDRAM)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ddram.ld")
endif()

if(CONFIG_SAMA5_BOOT_CS0FLASH)
  if(CONFIG_BOOT_SDRAM_DATA)
    set_property(GLOBAL PROPERTY LD_SCRIPT
                                 "${NUTTX_BOARD_DIR}/scripts/nor-ddram.ld")
  else()
    set_property(GLOBAL PROPERTY LD_SCRIPT
                                 "${NUTTX_BOARD_DIR}/scripts/nor-isram.ld")
  endif()
endif()
