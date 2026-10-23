/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dac.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_DAC_H
#define __ARCH_ARM_SRC_STM32H5_STM32_DAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_dac.h"

#include <nuttx/analog/dac.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Low-level ops helpers ****************************************************/

#define DAC_ENABLE(dac,d)                            \
        (dac)->llops->enable(dac,d)
#define DAC_WRITE_DRO(dac,d)                         \
        (dac)->llops->write_dro(dac,d)
#define DAC_DUMP_REGS(dac)                           \
        (dac)->llops->dump_regs(dac)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_STM32_DAC_LL_OPS

/* This structure provides the publicly visible representation of the
 * "lower-half" DAC driver structure.
 */

struct stm32_dac_dev_s
{
  /* Publicly visible portion of the "lower-half" DAC driver structure */

  const struct stm32_dac_ops_s *llops;

  /* Require cast-compatibility with private "lower-half" DAC structure */
};

/* Low-level operations for DAC */

struct stm32_dac_ops_s
{
  /* Enable / Disable DAC */

  void (*enable)(struct stm32_dac_dev_s *dev, bool enabled);

  /* Write DRO */

  void (*write_dro)(struct stm32_dac_dev_s *dev, uint16_t data);

  /* Dump DAC regs */

  void (*dump_regs)(struct stm32_dac_dev_s *dev);
};

#endif /* CONFIG_STM32_DAC_LL_OPS */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_dacinitialize
 *
 * Description:
 *   Initialize the DAC.  Interface 0 is DAC1 channel 1 and interface 1 is
 *   DAC1 channel 2.
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid dac device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct dac_dev_s;
struct dac_dev_s *stm32_dacinitialize(int intf);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_DAC_H */
