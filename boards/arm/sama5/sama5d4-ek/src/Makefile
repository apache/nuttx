############################################################################
# boards/arm/sama5/sama5d4-ek/src/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = sam_boot.c

ifeq ($(CONFIG_SAMA5_SPI0),y)
CSRCS += sam_spi.c
else
ifeq ($(CONFIG_SAMA5_SPI1),y)
CSRCS += sam_spi.c
endif
endif

ifeq ($(CONFIG_SAMA5_DDRCS),y)
CSRCS += sam_sdram.c
endif

ifeq ($(CONFIG_SAMA5_EBICS3_NAND),y)
CSRCS += sam_nandflash.c
endif

ifeq ($(CONFIG_SAMA5D4EK_DRAM_MAIN),y)
CSRCS += dram_main.c
endif

ifeq ($(CONFIG_SAMA5D4EK_AT25_MAIN),y)
CSRCS += at25_main.c
endif

ifeq ($(CONFIG_MTD_AT25),y)
ifeq ($(CONFIG_SAMA5_SPI0),y)
CSRCS += sam_at25.c
endif
endif

ifeq ($(CONFIG_FS_AUTOMOUNTER),y)
CSRCS += sam_automount.c
endif

ifeq ($(CONFIG_AUDIO_WM8904),y)
ifeq ($(CONFIG_SAMA5_TWI0),y)
ifeq ($(CONFIG_SAMA5_SSC0),y)
CSRCS += sam_wm8904.c
endif
endif
endif

ifeq ($(CONFIG_SAMA5_TWI0),y)
CSRCS += sam_pmic.c
endif

ifeq ($(CONFIG_AUDIO_NULL),y)
CSRCS += sam_audio_null.c
endif

ifeq ($(CONFIG_SAMA5_HSMCI0),y)
CSRCS += sam_hsmci.c
else
ifeq ($(CONFIG_SAMA5_HSMCI1),y)
CSRCS += sam_hsmci.c
endif
endif

ifeq ($(CONFIG_SAMA5_UHPHS),y)
CSRCS += sam_usb.c
else
ifeq ($(CONFIG_SAMA5_UDPHS),y)
CSRCS += sam_usb.c
endif
endif

ifeq ($(CONFIG_SAMA5_EMACB),y)
CSRCS += sam_ethernet.c
endif

ifeq ($(CONFIG_INPUT_MXT),y)
CSRCS += sam_maxtouch.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += sam_appinit.c sam_bringup.c
else
ifeq ($(CONFIG_BOARD_LATE_INITIALIZE),y)
CSRCS += sam_bringup.c
endif
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += sam_adc.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += sam_pwm.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += sam_usbmsc.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += sam_autoleds.c
else
CSRCS += sam_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += sam_buttons.c
endif

include $(TOPDIR)/boards/Board.mk
