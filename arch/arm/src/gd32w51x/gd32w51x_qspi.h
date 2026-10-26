/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_qspi.h
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

#ifndef __ARCH_ARM_SRC_GD32W51X_GD32W51X_QSPI_H
#define __ARCH_ARM_SRC_GD32W51X_GD32W51X_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spi/qspi.h>

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_qspi_initialize
 *
 * Description:
 *   Initialize the QSPI controller (indirect mode, polled) and return the
 *   lower half for the NOR flash drivers.  The board must define the
 *   GPIO_QSPI_SCK, GPIO_QSPI_CS and GPIO_QSPI_IO0..3 pin configurations.
 *
 * Input Parameters:
 *   intf - Interface number, must be zero
 *
 * Returned Value:
 *   The QSPI device on success, NULL on failure
 *
 ****************************************************************************/

struct qspi_dev_s *gd32_qspi_initialize(int intf);

/****************************************************************************
 * Name: gd32_qspi_enter_memorymapped
 *
 * Description:
 *   Put the QSPI controller into memory-mapped mode: the memory is read in
 *   place through the GD32W51X_QSPIMEM_BASE window (0x90000000).  The QSPI
 *   command and memory operations fail with -EBUSY until
 *   gd32_qspi_exit_memorymapped().  Only data can be used in place: code
 *   run from the window hangs the bus.
 *
 * Input Parameters:
 *   dev     - QSPI device
 *   meminfo - The read transfer used to fetch the memory
 *   lpto    - Number of SCK cycles without access after which CS is
 *             released, or 0 to keep it asserted
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_qspi_enter_memorymapped(struct qspi_dev_s *dev,
                                  const struct qspi_meminfo_s *meminfo,
                                  uint32_t lpto);

/****************************************************************************
 * Name: gd32_qspi_exit_memorymapped
 *
 * Description:
 *   Take the QSPI controller out of memory-mapped mode.
 *
 * Input Parameters:
 *   dev - QSPI device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gd32_qspi_exit_memorymapped(struct qspi_dev_s *dev);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32W51X_GD32W51X_QSPI_H */
