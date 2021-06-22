############################################################################
# boards/risc-v/jh7100/smartl-jh7100/src/Makefile
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

CSRCS  = jh7100_bringup.c jh7100_boot.c

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += jh7100_appinit.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += jh7100_leds.c
endif

ifeq ($(CONFIG_ARCH_FPU),y)
CSRCS += jh7100_ostest.c
endif

include $(TOPDIR)/boards/Board.mk
