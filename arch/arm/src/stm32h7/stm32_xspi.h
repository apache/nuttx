/****************************************************************************
 * arch/arm/src/stm32h7/stm32_xspi.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_XSPI_H
#define __ARCH_ARM_SRC_STM32H7_STM32_XSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/spi/qspi.h>

#ifdef CONFIG_STM32_XSPI

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum stm32_xspi_memtype_e
{
  STM32_XSPI_MEMTYPE_MICRON       = 0,
  STM32_XSPI_MEMTYPE_MACRONIX     = 1,
  STM32_XSPI_MEMTYPE_APMEM        = 2,
  STM32_XSPI_MEMTYPE_MACRONIX_RAM = 3,
  STM32_XSPI_MEMTYPE_HYPERBUS     = 4,
  STM32_XSPI_MEMTYPE_APMEM_16BIT  = 6
};

enum stm32_xspi_iomport_e
{
  STM32_XSPI_IOM_PORT1 = 0,
  STM32_XSPI_IOM_PORT2 = 1
};

enum stm32_xspi_cs_e
{
  STM32_XSPI_CS_DISABLED = 0,
  STM32_XSPI_CS_NCS1,
  STM32_XSPI_CS_NCS2
};

struct stm32_xspi_config_s
{
  enum stm32_xspi_memtype_e memtype;
  enum stm32_xspi_iomport_e iomport;
  enum stm32_xspi_cs_e cs_override;
  uint8_t devsize;
  uint8_t csht;
  uint8_t prescaler;
  uint8_t fthres;
  uint8_t wrapsize;
  uint8_t maxtran;
  uint8_t csbound;
  uint16_t req2ack;
  uint32_t refresh;
  bool clock_mode3;
  bool free_running;
  bool sample_shift;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_xspi_initialize
 *
 * Description:
 *   Initialize an XSPI controller.
 *
 * Input Parameters:
 *   intf   - XSPI controller number (1 or 2)
 *   config - XSPI controller configuration
 *
 * Returned Value:
 *   Valid QSPI device structure on success; NULL on failure.
 *
 ****************************************************************************/

struct qspi_dev_s *
stm32_xspi_initialize(int intf, const struct stm32_xspi_config_s *config);

/****************************************************************************
 * Name: stm32_xspi_enter_memorymapped
 *
 * Description:
 *   Configure the XSPI controller for memory-mapped access.
 *
 * Input Parameters:
 *   dev       - QSPI device
 *   readinfo  - Memory transfer parameters used for reading
 *   writeinfo - Memory transfer parameters used for writing; may be NULL
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_xspi_enter_memorymapped(
  struct qspi_dev_s *dev,
  const struct qspi_meminfo_s *readinfo,
  const struct qspi_meminfo_s *writeinfo);

/****************************************************************************
 * Name: stm32_xspi_exit_memorymapped
 *
 * Description:
 *   Take the XSPI device out of memory-mapped mode.
 *
 * Input Parameters:
 *   dev - QSPI device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_xspi_exit_memorymapped(struct qspi_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32_XSPI */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_XSPI_H */
