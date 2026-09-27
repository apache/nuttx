############################################################################
# boards/arm/imxrt/imxrt1180-evk/src/Makefile
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

CSRCS  = imxrt_boot.c imxrt_bringup.c imxrt_appinit.c

# On the M33 target the ROM boots our image directly, so we must supply the
# FlexSPI Configuration Block ourselves.  The M7 image is a raw XIP payload
# and carries no flash headers.

ifeq ($(CONFIG_ARCH_CORTEXM33),y)
CSRCS += imxrt_flash_headers.c
endif

ifeq ($(CONFIG_IMXRT1180_EVK_BOOTLOADER),y)
CSRCS += imxrt_bootloader.c
endif

ifeq ($(CONFIG_USBDEV_DMAMEMORY),y)
CSRCS += imxrt_dma_alloc.c
endif

# TRDC descriptors can only be programmed by the core that owns them, which
# is the M33 (CONFIG_IMXRT_TRDC is selected only by the M33 chip choice).
# There is no weak fallback for the board hook: if TRDC is enabled the board
# must supply an access policy, or the link fails.

ifeq ($(CONFIG_IMXRT_TRDC),y)
endif

include $(TOPDIR)/boards/Board.mk
