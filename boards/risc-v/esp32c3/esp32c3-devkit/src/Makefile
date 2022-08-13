#############################################################################
# boards/risc-v/esp32c3/esp32c3-devkit/src/Makefile
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
#############################################################################

include $(TOPDIR)/Make.defs

SCRIPTDIR = $(BOARD_DIR)$(DELIM)scripts

CONFIGFILE = $(TOPDIR)$(DELIM)include$(DELIM)nuttx$(DELIM)config.h

CSRCS = esp32c3_boot.c esp32c3_bringup.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += esp32c3_appinit.c
ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += esp32c3_reset.c
endif

endif

ifeq ($(CONFIG_ESP32C3_SPIFLASH),y)
CSRCS += esp32c3_spiflash.c
endif

ifeq ($(CONFIG_ESP32C3_WIRELESS),y)
CSRCS += esp32c3_board_wlan.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += esp32c3_gpio.c
endif

ifeq ($(CONFIG_ESP32C3_SPI),y)
CSRCS += esp32c3_board_spi.c
endif

ifeq ($(CONFIG_SPI_DRIVER),y)
CSRCS += esp32c3_board_spidev.c
endif

ifeq ($(CONFIG_SPI_SLAVE_DRIVER),y)
CSRCS += esp32c3_board_spislavedev.c
endif

ifeq ($(CONFIG_I2C_DRIVER),y)
CSRCS += esp32c3_i2c.c
endif

ifeq ($(CONFIG_CAN),y)
CSRCS += esp32c3_twai.c
endif

ifeq ($(CONFIG_SENSORS_BMP180),y)
CSRCS += esp32c3_bmp180.c
endif

ifeq ($(CONFIG_WATCHDOG),y)
CSRCS += esp32c3_wdt.c
endif

ifeq ($(CONFIG_ONESHOT),y)
CSRCS += esp32c3_oneshot.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += esp32c3_ledc.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += esp32c3_adc.c
endif

ifeq ($(CONFIG_LCD_ST7735),y)
CSRCS += esp32c3_st7735.c
endif

ifeq ($(CONFIG_LCD_ST7789),y)
CSRCS += esp32c3_st7789.c
endif

ifeq ($(CONFIG_LCD_GC9A01),y)
CSRCS += esp32c3_gc9a01.c
endif

ifeq ($(CONFIG_BOARDCTL_IOCTL),y)
CSRCS += esp32c3_ioctl.c
endif

ifeq ($(CONFIG_BOARDCTL_UNIQUEID),y)
CSRCS += esp32c3_uid.c
endif

ifeq ($(CONFIG_NSH_ROMFSETC),y)
ifneq ($(CONFIG_NSH_CUSTOMROMFS),y)
  RCSRCS = etc/init.d/rc.sysinit etc/init.d/rcS
  RCRAWS = etc/group etc/passwd
endif
endif

ifeq ($(CONFIG_BUILD_PROTECTED),y)
SCRIPTIN = $(SCRIPTDIR)$(DELIM)protected.template.ld
else
SCRIPTIN = $(SCRIPTDIR)$(DELIM)flat.template.ld
endif
SCRIPTOUT = $(SCRIPTDIR)$(DELIM)esp32c3_out.ld

.PHONY = context distclean

$(SCRIPTOUT): $(SCRIPTIN) $(CONFIGFILE)
	$(Q) $(CC) -isystem $(TOPDIR)/include -I $(BOARD_DIR)/scripts -C -P -x c -E $(SCRIPTIN) -o $@

context:: $(SCRIPTOUT)

distclean::
	$(call DELFILE, $(SCRIPTOUT))

include $(TOPDIR)/boards/Board.mk
