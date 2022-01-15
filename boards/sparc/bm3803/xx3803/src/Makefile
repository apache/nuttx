############################################################################
# boards/sparc/bm3803/xx3803/src/Makefile
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

-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS = bm3803_boot.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += bm3803_appinit.c
ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += bm3803_reset.c
endif
endif

ifeq ($(CONFIG_MTD_AM29LV),y)
CSRCS += bm3803_am29lv.c
endif

ifeq ($(CONFIG_XX3803_WDG),y)
CSRCS += bm3803_wdt.c
endif

include $(TOPDIR)/boards/Board.mk
