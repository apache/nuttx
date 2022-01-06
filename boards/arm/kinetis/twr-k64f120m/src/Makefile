############################################################################
# boards/arm/kinetis/twr-k64f120m/src/Makefile
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

#CSRCS = k64_boot.c k64_spi.c
CSRCS = k64_boot.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += k64_leds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
#CSRCS += k64_buttons.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += k64_appinit.c
endif

ifeq ($(CONFIG_KINETIS_SDHC),y)
CSRCS += k64_sdhc.c
ifeq ($(CONFIG_FS_AUTOMOUNTER),y)
CSRCS += k64_automount.c
endif
endif

ifeq ($(CONFIG_USBDEV),y)
#CSRCS += k64_usbdev.c
endif

ifeq ($(CONFIG_USBMSC),y)
#CSRCS += k64_usbmsc.c
endif

include $(TOPDIR)/boards/Board.mk
