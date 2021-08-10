############################################################################
# boards/arm/str71x/olimex-strp711/src/Makefile
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

CSRCS = str71_spi.c str71_leds.c str71_buttons.c
ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += str71_appinit.c
endif
ifeq ($(CONFIG_ENC28J60),y)
CSRCS += str71_enc28j60.c
endif

include $(TOPDIR)/boards/Board.mk
