/****************************************************************************
 * arch/arm/src/samv7/sam_serial_spi.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_SERIAL_SPI_H
#define __ARCH_ARM_SRC_SAMV7_SAM_SERIAL_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "sam_config.h"

#ifdef CONFIG_SAMV7_USART_IS_SPI_MASTER

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
 * Name: sam_serial_spi_initialize
 *
 * Description:
 *   Initialize the selected SPI port in master mode
 *
 * Input Parameters:
 *   port - USART interface to be used (0-2)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s;
struct spi_dev_s *sam_serial_spi_initialize(int port);

/****************************************************************************
 * Name: sam_serial_status
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *
 * Returned Value:
 *   Bit-encoded SPI status (see include/nuttx/spi/spi.h).
 *
 ****************************************************************************/

uint8_t sam_serial_status(struct spi_dev_s *dev, uint32_t devid);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SAMV7_USART_IS_SPI_MASTER */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_SERIAL_SPI_H */
