/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_qspi.h
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_QSPI_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"

#ifdef CONFIG_STM32L4_QSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

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
 * Name: stm32l4_qspi_initialize
 *
 * Description:
 *   Initialize the selected QSPI port in master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct qspi_dev_s;
struct qspi_dev_s *stm32l4_qspi_initialize(int intf);

/****************************************************************************
 * Name: stm32l4_qspi_enter_memorymapped
 *
 * Description:
 *   Put the QSPI device into memory mapped mode
 *
 * Input Parameters:
 *   dev - QSPI device
 *   meminfo - parameters like for a memory transfer used for reading
 *   lpto - number of cycles to wait to automatically de-assert CS
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32l4_qspi_enter_memorymapped(struct qspi_dev_s *dev,
                                     const struct qspi_meminfo_s *meminfo,
                                     uint32_t lpto);

/****************************************************************************
 * Name: stm32l4_qspi_exit_memorymapped
 *
 * Description:
 *   Take the QSPI device out of memory mapped mode
 *
 * Input Parameters:
 *   dev - QSPI device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32l4_qspi_exit_memorymapped(struct qspi_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32L4_QSPI */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_QSPI_H */
