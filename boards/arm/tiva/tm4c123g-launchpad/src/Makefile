############################################################################
# boards/arm/tiva/tm4c123g-launchpad/src/Makefile
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

CSRCS = tm4c_boot.c tm4c_bringup.c tm4c_ssi.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += tm4c_autoleds.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += tm4c_appinit.c
endif

ifeq ($(CONFIG_TIVA_TIMER),y)
CSRCS += tm4c_timer.c
endif

ifeq ($(CONFIG_TIVA_ADC),y)
CSRCS += tm4c_adc.c
endif

ifeq ($(CONFIG_TIVA_CAN),y)
  CSRCS += tm4c_can.c
endif

ifeq ($(CONFIG_CAN_MCP2515),y)
  CSRCS += tm4c_mcp2515.c
endif

ifeq ($(CONFIG_MTD_AT24XX),y)
ifeq ($(CONFIG_TIVA_I2C0),y)
CSRCS += tm4c_at24.c
endif
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += tm4c_buttons.c
endif

include $(TOPDIR)/boards/Board.mk
