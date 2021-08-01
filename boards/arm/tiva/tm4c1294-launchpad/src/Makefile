############################################################################
# boards/arm/tiva/tm4c1294-launchpad/src/Makefile
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

CSRCS = tm4c_boot.c tm4c_bringup.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += tm4c_autoleds.c
else
CSRCS += tm4c_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += tm4c_buttons.c
endif

ifeq ($(CONFIG_TIVA_ETHERNET),y)
CSRCS += tm4c_ethernet.c
endif

ifeq ($(CONFIG_DK_TM4C129X_TIMER),y)
CSRCS += tm4c_timer.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += tm4c_appinit.c
endif

ifeq ($(CONFIG_TIVA_HCIUART),y)
ifeq ($(CONFIG_BLUETOOTH_UART),y)
CSRCS += tm4c_hciuart.c
endif
endif

include $(TOPDIR)/boards/Board.mk
