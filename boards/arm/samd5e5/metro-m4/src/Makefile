############################################################################
# boards/arm/samd5e5/metro-m4/src/Makefile
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

CSRCS = sam_boot.c sam_bringup.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += sam_appinit.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += sam_autoleds.c
else
CSRCS += sam_userleds.c
endif

ifeq ($(CONFIG_USBDEV),y)
CSRCS += sam_usbdev.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += sam_usbmsc.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS += sam_composite.c
endif

ifeq ($(CONFIG_USBHOST),y)
CSRCS += sam_usbhost.c
endif

ifeq ($(CONFIG_FS_AUTOMOUNTER),y)
CSRCS += sam_automount.c
endif

ifeq ($(CONFIG_SAMD5E5_SERCOM5_ISI2C),y)
CSRCS += sam_i2c.c
ifeq ($(CONFIG_MTD_AT24XX),y)
CSRCS += sam_at24.c
endif
endif

ifeq ($(CONFIG_FS_SMARTFS),y)
CSRCS += sam_smartfs.c
endif

ifeq ($(CONFIG_BQ27426),y)
CSRCS += sam_bq27426.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += sam_gpio.c
endif

include $(TOPDIR)/boards/Board.mk
