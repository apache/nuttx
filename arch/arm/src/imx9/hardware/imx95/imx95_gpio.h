/****************************************************************************
 * arch/arm/src/imx9/hardware/imx95/imx95_gpio.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_GPIO_H
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "imx95_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX9_GPIO_VERID_OFFSET          (0x0000) /* Version ID */
#define IMX9_GPIO_PARAM_OFFSET          (0x0004) /* Parameter */
#define IMX9_GPIO_LOCK_OFFSET           (0x000c) /* Lock */
#define IMX9_GPIO_PCNS_OFFSET           (0x0010) /* Pin Control Nonsecure */
#define IMX9_GPIO_ICNS_OFFSET           (0x0014) /* Interrupt Control Nonsecure */
#define IMX9_GPIO_PCNP_OFFSET           (0x0018) /* Pin Control Nonprivilege */
#define IMX9_GPIO_ICNP_OFFSET           (0x001c) /* Interrupt Control Nonprivilege */
#define IMX9_GPIO_PDOR_OFFSET           (0x0040) /* Port Data Output */
#define IMX9_GPIO_PSOR_OFFSET           (0x0044) /* Port Set Output */
#define IMX9_GPIO_PCOR_OFFSET           (0x0048) /* Port Clear Output */
#define IMX9_GPIO_PTOR_OFFSET           (0x004c) /* Port Toggle Output */
#define IMX9_GPIO_PDIR_OFFSET           (0x0050) /* Port Data Input */
#define IMX9_GPIO_PDDR_OFFSET           (0x0054) /* Port Data Direction */
#define IMX9_GPIO_PIDR_OFFSET           (0x0058) /* Port Input Disable */
#define IMX9_GPIO_P0DR_OFFSET           (0x0060) /* Pin Data (0-31 at offsets of n * 4h) */
#define IMX9_GPIO_ICR0_OFFSET           (0x0080) /* Interrupt Control (0-31 at offsets of n * 4h) */
#define IMX9_GPIO_GICLR_OFFSET          (0x0100) /* Global Interrupt Control Low */
#define IMX9_GPIO_GICHR_OFFSET          (0x0104) /* Global Interrupt Control High */
#define IMX9_GPIO_ISFR0_OFFSET          (0x0120) /* Interrupt Status Flag */
#define IMX9_GPIO_ISFR1_OFFSET          (0x0124) /* Interrupt Status Flag */

#endif /* __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_GPIO_H */
