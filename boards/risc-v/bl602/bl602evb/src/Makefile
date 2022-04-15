############################################################################
# boards/risc-v/bl602/bl602evb/src/Makefile
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

CSRCS  = bl602_bringup.c bl602_boot.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += bl602_appinit.c
ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += bl602_reset.c
endif
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += bl602_gpio.c
endif

include $(TOPDIR)/boards/Board.mk
