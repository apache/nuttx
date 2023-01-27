############################################################################
# boards/arm64/a64/pinephone/src/Makefile
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

CSRCS = pinephone_boardinit.c pinephone_bringup.c pinephone_pmic.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += pinephone_appinit.c
ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += pinephone_reset.c
endif
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += pinephone_autoleds.c
endif

ifeq ($(CONFIG_USERLED),y)
CSRCS += pinephone_userleds.c
endif

ifeq ($(CONFIG_VIDEO_FB),y)
CSRCS += pinephone_display.c pinephone_lcd.c
endif

ifeq ($(CONFIG_INPUT_GT9XX),y)
CSRCS += pinephone_touch.c
endif

include $(TOPDIR)/boards/Board.mk
