############################################################################
# boards/arm/kinetis/freedom-k64f/src/Makefile
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

CSRCS = k64_boot.c k64_spi.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += k64_autoleds.c
else
CSRCS += k64_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += k64_buttons.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += k64_appinit.c k64_bringup.c
else ifeq ($(CONFIG_BOARD_LATE_INITIALIZE),y)
CSRCS += k64_bringup.c
endif

ifeq ($(CONFIG_I2C),y)
CSRCS += k64_i2c.c
endif

ifeq ($(CONFIG_KINETIS_SDHC),y)
CSRCS += k64_sdhc.c
ifeq ($(CONFIG_FS_AUTOMOUNTER),y)
CSRCS += k64_automount.c
endif
endif

ifeq ($(CONFIG_USBDEV),y)
CSRCS += k64_usbdev.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += k64_usbmsc.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += k64_pwm.c
endif

ifeq ($(CONFIG_BOARDCTL_UNIQUEID),y)
CSRCS += k64_uid.c
endif

include $(TOPDIR)/boards/Board.mk
