############################################################################
# boards/sim/sim/sim/src/Makefile
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

# Add dummy.c to ensure that we have at least one object.
# On some platforms like macOS, we can't create an empty archive.
CSRCS = dummy.c

ifeq ($(CONFIG_BOARD_LATE_INITIALIZE),y)
  CSRCS += sim_boot.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
  CSRCS += sim_appinit.c
endif

ifeq ($(CONFIG_BOARD_LATE_INITIALIZE),y)
  CSRCS += sim_bringup.c
ifeq ($(CONFIG_LIBC_ZONEINFO_ROMFS),y)
  CSRCS += sim_zoneinfo.c
endif
else ifeq ($(CONFIG_BOARDCTL),y)
  CSRCS += sim_bringup.c
ifeq ($(CONFIG_LIBC_ZONEINFO_ROMFS),y)
  CSRCS += sim_zoneinfo.c
endif
endif

ifeq ($(CONFIG_EXAMPLES_GPIO),y)
ifeq ($(CONFIG_GPIO_LOWER_HALF),y)
  CSRCS += sim_ioexpander.c
else
  CSRCS += sim_gpio.c
endif
endif

ifeq ($(CONFIG_NSH_ROMFSETC),y)
ifneq ($(CONFIG_NSH_CUSTOMROMFS),y)
  RCSRCS = etc/init.d/rc.sysinit etc/init.d/rcS
  RCRAWS = etc/group etc/passwd
endif
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
  CSRCS += sim_buttons.c
endif

ifeq ($(CONFIG_MOTOR_FOC_DUMMY),y)
CSRCS += sim_foc.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS += sim_composite.c
endif

include $(TOPDIR)/boards/Board.mk
