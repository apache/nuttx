############################################################################
# boards/arm/lpc214x/mcu123-lpc214x/src/Makefile
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

CSRCS = lpc2148_spi1.c lpc2148_leds.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += lpc2148_appinit.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += lpc2148_usbmsc.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS += lpc2148_composite.c
endif

include $(TOPDIR)/boards/Board.mk
