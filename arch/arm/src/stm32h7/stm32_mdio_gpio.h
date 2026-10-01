/****************************************************************************
 * arch/arm/src/stm32h7/stm32_mdio_gpio.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_MDIO_GPIO_H
#define __ARCH_ARM_SRC_STM32H7_STM32_MDIO_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/mdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mdiogpio_bus_initialize
 *
 * Description:
 *   Initialize the MDIO bus to emilate STA via GPIO pins
 *
 * Input Parameters:
 *   mdc_out  - MDC GPIO pin config for output
 *   mdio_out - MDIO GPIO pin config for output
 *   mdio_in  - MDIO GPIO pin config for input
 *
 * Returned Value:
 *   Initialized MDIO GPIO bus structure or NULL on failure
 *
 ****************************************************************************/

struct mdio_bus_s *stm32_mdiogpio_bus_initialize(uint32_t mdc_out,
                                                 uint32_t mdio_out,
                                                 uint32_t mdio_inp);

#endif /* __ARCH_ARM_SRC_STM32H7_STM32_MDIO_GPIO_H */
