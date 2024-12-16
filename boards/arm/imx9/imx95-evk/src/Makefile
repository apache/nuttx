############################################################################
# boards/arm/imx95/imx95-evk/src/Makefile
#
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2024 NXP
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

CSRCS = imx95_boardinit.c imx95_bringup.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += imx95_appinit.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += imx95_pwm.c
endif

ifeq ($(CONFIG_IMX9_LPI2C),y)
CSRCS += imx95_i2c.c
endif

ifeq ($(CONFIG_IMX9_LPSPI),y)
CSRCS += imx95_spi.c
endif

include $(TOPDIR)/boards/Board.mk
