############################################################################
# boards/arm/lpc55xx/lpcxpresso-lpc55s69/src/Makefile
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

CSRCS = lpc55_boot.c lpc55_bringup.c
# lpc55_userleds.c

ifeq ($(CONFIG_ARCH_LEDS),y)
#CSRCS += lpc55_autoleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
#CSRCS += lpc55_buttons.c
endif

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += lpc55_appinit.c
endif

ifeq ($(CONFIG_LPC54_HAVE_I2C_MASTER),y)
CSRCS += lpc55_i2c.c
endif

ifeq ($(CONFIG_SYSTEM_I2CTOOL),y)
CSRCS += lpc55_i2ctool.c
endif

include $(TOPDIR)/boards/Board.mk
