/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_syscfg.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_SYSCFG_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#if defined(CONFIG_GD32F4_GD32F4XX)
#  include "hardware/gd32f4xx_syscfg.h"
#else
#  error "Unknown GD32 Chip"
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_syscfg_bootmode_config
 *
 * Description:
 *   This function configure the boot mode.
 *
 ****************************************************************************/

void gd32_syscfg_bootmode_config(uint8_t syscfg_bootmode);

/****************************************************************************
 * Name: gd32_syscfg_fmc_swap_config
 *
 * Description:
 *   This function configure FMC memory mapping swap.
 *
 ****************************************************************************/

void gd32_syscfg_fmc_swap_config(uint32_t syscfg_fmc_swap);

/****************************************************************************
 * Name: gd32_syscfg_exmc_swap_config
 *
 * Description:
 *   This function configure EXMC memory mapping swap.
 *
 ****************************************************************************/

void gd32_syscfg_exmc_swap_config(uint32_t syscfg_exmc_swap);

/****************************************************************************
 * Name: gd32_syscfg_exti_line_config
 *
 * Description:
 *   This function configure the GPIO pin as EXTI Line.
 *
 ****************************************************************************/

void gd32_syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin);

/****************************************************************************
 * Name: gd32_syscfg_enet_phy_interface_config
 *
 * Description:
 *   This function configure the PHY interface for the ethernet MAC.
 *
 ****************************************************************************/

void
gd32_syscfg_enet_phy_interface_config(uint32_t syscfg_enet_phy_interface);

/****************************************************************************
 * Name: gd32_syscfg_compensation_config
 *
 * Description:
 *   This function configure the I/O compensation cell.
 *
 ****************************************************************************/

void gd32_syscfg_compensation_config(uint32_t syscfg_compensation);

/****************************************************************************
 * Name: gd32_syscfg_flag_get
 *
 * Description:
 *   This function checks whether the I/O compensation cell ready flag
 *   is set or not.
 *
 ****************************************************************************/

bool gd32_syscfg_flag_get(void);

/****************************************************************************
 * Name: gd32_syscfg_clock_enable
 *
 * Description:
 *   Enable SYSCFG clock
 ****************************************************************************/

void gd32_syscfg_clock_enable(void);

/****************************************************************************
 * Name: gd32_syscfg_clock_disable
 *
 * Description:
 *   Dinable SYSCFG clock
 ****************************************************************************/

void gd32_syscfg_clock_disable(void);

#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_SYSCFG_H */
