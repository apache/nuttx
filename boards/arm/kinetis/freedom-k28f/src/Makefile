############################################################################
# boards/arm/kinetis/freedom-k28f/src/Makefile
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

CSRCS = k28_boot.c k28_bringup.c k28_spi.c k28_i2c.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += k28_appinit.c
endif

# If the RGB driver is not enabled, then treat the RGB as 3 LEDs

ifneq ($(CONFIG_RGBLED),y)
ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += k28_autoleds.c
else
CSRCS += k28_userleds.c
endif
endif

ifeq ($(CONFIG_KINETIS_SDHC),y)
CSRCS += k28_sdhc.c
ifeq ($(CONFIG_FS_AUTOMOUNTER),y)
CSRCS += k28_automount.c
endif
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += k28_pwm.c
endif

ifeq ($(CONFIG_KINETIS_USBOTG),y)
CSRCS += k28_usbdev.c
endif

ifeq ($(CONFIG_KINETIS_USBHS),y)
ifeq ($(CONFIG_USBHOST),y)
CSRCS += k28_usbhshost.c
endif
endif

include $(TOPDIR)/boards/Board.mk
