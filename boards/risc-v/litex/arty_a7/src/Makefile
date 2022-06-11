############################################################################
# boards/risc-v/litex/arty_a7/src/Makefile
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

CSRCS  = litex_bringup.c litex_boot.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += litex_appinit.c
endif

ifeq ($(CONFIG_LITEX_SDIO),y)
CSRCS += litex_sdio.c
endif

ifeq ($(CONFIG_FS_AUTOMOUNTER),y)
CSRCS += litex_automount.c
endif

include $(TOPDIR)/boards/Board.mk
