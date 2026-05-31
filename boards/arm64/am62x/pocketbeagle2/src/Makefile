############################################################################
# boards/arm64/am62x/pocketbeagle2/src/Makefile
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

# Board-specific C sources (always compiled)

CSRCS = pocketbeagle2_boardinit.c pocketbeagle2_bringup.c

# Automatic LED support

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += pocketbeagle2_autoleds.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += pocketbeagle2_gpio.c
endif

include $(TOPDIR)/boards/Board.mk
