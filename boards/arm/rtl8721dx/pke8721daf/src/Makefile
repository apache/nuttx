############################################################################
# boards/arm/rtl8721dx/pke8721daf/src/Makefile
#
# SPDX-License-Identifier: Apache-2.0
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

CSRCS = rtl8721dx_boot.c rtl8721dx_bringup.c

ifeq ($(CONFIG_AMEBA_GPIO),y)
CSRCS += rtl8721dx_gpio.c

# The board pin table pulls in the shared driver's public header from
# arch/arm/src/common/ameba/, which is not on the default board include path.

CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)arch$(DELIM)arm$(DELIM)src$(DELIM)common$(DELIM)ameba
endif

ifeq ($(CONFIG_AMEBA_UART),y)
CSRCS += rtl8721dx_uart.c

# The board UART table pulls in the shared driver's public header from
# arch/arm/src/common/ameba/, which is not on the default board include path.

CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)arch$(DELIM)arm$(DELIM)src$(DELIM)common$(DELIM)ameba
endif

include $(TOPDIR)/boards/Board.mk
