#############################################################################
# boards/risc-v/esp32c3/esp32c3-devkit-rust-1/src/Makefile
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

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += esp32c3_autoleds.c
endif

ifeq ($(CONFIG_BOARDCTL_IOCTL),y)
CSRCS += esp32c3_ioctl.c
endif

ifeq ($(CONFIG_ESP32C3_SPI),y)
CSRCS += esp32c3_board_spi.c
endif

ifeq ($(CONFIG_SENSORS_APDS9960),y)
CSRCS += esp32c3_apds9960.c
endif

ifeq ($(CONFIG_LCD_APA102),y)
CSRCS += esp32c3_apa102.c
endif

SCRIPTIN = $(SCRIPTDIR)$(DELIM)flat.template.ld
SCRIPTOUT = $(SCRIPTDIR)$(DELIM)esp32c3_out.ld

.PHONY = context distclean

$(SCRIPTOUT): $(SCRIPTIN) $(CONFIGFILE)
	$(Q) $(CC) -isystem $(TOPDIR)/include -I $(BOARD_DIR)/scripts -C -P -x c -E $(SCRIPTIN) -o $@

context:: $(SCRIPTOUT)

distclean::
	$(call DELFILE, $(SCRIPTOUT))

include $(TOPDIR)/boards/Board.mk
