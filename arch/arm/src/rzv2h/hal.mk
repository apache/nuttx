############################################################################
# arch/arm/src/rzv2h/hal.mk
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

############################################################################
# Define hal paths
############################################################################
HAL_INDIR := common$(DELIM)renesas$(DELIM)hal_renesas

############################################################################
# Include header paths of BSP
############################################################################
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)inc
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)inc$(DELIM)instances
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)inc$(DELIM)instances$(DELIM)rzv
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)inc$(DELIM)api
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)bsp$(DELIM)mcu$(DELIM)rzv2h
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)bsp$(DELIM)mcu$(DELIM)rzv2h$(DELIM)cr
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)bsp$(DELIM)mcu$(DELIM)all
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)bsp$(DELIM)mcu$(DELIM)all$(DELIM)cr
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)bsp$(DELIM)cmsis$(DELIM)Device$(DELIM)RENESAS$(DELIM)Include
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)nuttx$(DELIM)rz$(DELIM)rz_cfg$(DELIM)fsp_cfg
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)nuttx$(DELIM)rz$(DELIM)rz_cfg$(DELIM)fsp_cfg$(DELIM)bsp
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)nuttx$(DELIM)rz$(DELIM)rz_cfg$(DELIM)fsp_cfg$(DELIM)bsp$(DELIM)rzv2h
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)nuttx$(DELIM)rz$(DELIM)rz_cfg$(DELIM)fsp_cfg$(DELIM)bsp$(DELIM)rzv2h$(DELIM)cr
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)nuttx$(DELIM)rz$(DELIM)rz_cfg$(DELIM)fsp_cfg$(DELIM)rzv
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)nuttx$(DELIM)rz$(DELIM)portable$(DELIM)rzv
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)nuttx$(DELIM)rz$(DELIM)portable$(DELIM)rzv$(DELIM)cr
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)nuttx$(DELIM)cmsis
INCLUDES += $(INCDIR_PREFIX)$(HAL_INDIR)$(DELIM)nuttx$(DELIM)cmsis$(DELIM)include$(DELIM)cr

############################################################################
# Include header paths of XXX module
############################################################################

############################################################################
# BSP source code from hal driver
############################################################################
CHIP_CSRCS += $(HAL_INDIR)$(DELIM)nuttx$(DELIM)cmsis$(DELIM)src$(DELIM)cr$(DELIM)system_cr.c
CHIP_CSRCS += $(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)bsp$(DELIM)mcu$(DELIM)all$(DELIM)bsp_clocks.c
CHIP_CSRCS += $(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)bsp$(DELIM)mcu$(DELIM)all$(DELIM)bsp_io.c
CHIP_CSRCS += $(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)bsp$(DELIM)mcu$(DELIM)all$(DELIM)bsp_address_convert.c
CHIP_CSRCS += $(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)bsp$(DELIM)mcu$(DELIM)all$(DELIM)bsp_delay.c
CHIP_CSRCS += $(HAL_INDIR)$(DELIM)nuttx$(DELIM)rz$(DELIM)portable$(DELIM)rzv$(DELIM)cr$(DELIM)bsp_irq.c

############################################################################
# hal driver
############################################################################
ifeq ($(CONFIG_DEV_GPIO),y)
CHIP_CSRCS += $(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)r_ioport$(DELIM)r_ioport.c
endif

ifeq ($(CONFIG_RZV2H_SCI_B),y)
CHIP_CSRCS += $(HAL_INDIR)$(DELIM)drivers$(DELIM)rz$(DELIM)fsp$(DELIM)src$(DELIM)rzv$(DELIM)r_sci_b_uart$(DELIM)r_sci_b_uart.c
endif

############################################################################
