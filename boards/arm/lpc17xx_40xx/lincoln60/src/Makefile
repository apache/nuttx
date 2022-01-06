############################################################################
# boards/arm/lpc17xx_40xx/lincoln60/src/Makefile
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

CSRCS = lpc17_40_boot.c lpc17_40_leds.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += lpc17_40_appinit.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += lpc17_40_usbmsc.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += lpc17_40_buttons.c
endif

include $(TOPDIR)/boards/Board.mk
