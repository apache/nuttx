/****************************************************************************
 * arch/arm/include/nrf54l/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_NRF54L_IRQ_H
#define __ARCH_ARM_INCLUDE_NRF54L_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF54L_IRQ_RESERVED     0
#define NRF54L_IRQ_NMI          2
#define NRF54L_IRQ_HARDFAULT    3
#define NRF54L_IRQ_MEMFAULT     4
#define NRF54L_IRQ_BUSFAULT     5
#define NRF54L_IRQ_USAGEFAULT   6
#define NRF54L_IRQ_SECUREFAULT  7
#define NRF54L_IRQ_SVCALL       11
#define NRF54L_IRQ_DBGMONITOR   12
#define NRF54L_IRQ_PENDSV       14
#define NRF54L_IRQ_SYSTICK      15
#define NRF54L_IRQ_EXTINT       16

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/nrf54l/nrf54l_irq.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);
#endif

#endif /* __ARCH_ARM_INCLUDE_NRF54L_IRQ_H */
