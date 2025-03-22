/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_rcc.h
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

#ifndef ARCH_RISC_V_SRC_MPFS_MPFS_RCC_H
#define ARCH_RISC_V_SRC_MPFS_MPFS_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum mpfs_rcc_id_e
{
  MPFS_RCC_CAN,
  MPFS_RCC_I2C,
  MPFS_RCC_PWM,
  MPFS_RCC_SPI,
  MPFS_RCC_UART,
  MPFS_RCC_LAST,
};

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_set_reset
 *
 * Description:
 *   Enable / disable peripheral reset.
 *
 * Input Parameters:
 *   rcc_id   - Device id.
 *   instance - Optional instance number for device.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MPFS_RCC
int mpfs_set_reset(int rcc_id, int instance, bool state);
#else
#  define mpfs_set_reset(rcc_id, instance, state) (0)
#endif

/****************************************************************************
 * Name: mpfs_set_clock
 *
 * Description:
 *   Enable / disable peripheral clock.
 *
 * Input Parameters:
 *   rcc_id   - Device id.
 *   instance - Optional instance number for device.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_MPFS_RCC
int mpfs_set_clock(int rcc_id, int instance, bool state);
#else
#  define mpfs_set_clock(rcc_id, instance, state) (0)
#endif

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* ARCH_RISC_V_SRC_MPFS_MPFS_RCC_H */
