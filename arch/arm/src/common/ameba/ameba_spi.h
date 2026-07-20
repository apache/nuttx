/****************************************************************************
 * arch/arm/src/common/ameba/ameba_spi.h
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_SPI_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Ameba SPI (DesignWare SSI) controllers exposed to NuttX as SPI master
 * buses.  The CLK/MOSI/MISO/CS pads are given with the same AMEBA_PA()/
 * AMEBA_PB() PinName code used by the GPIO driver (see ameba_gpio.h); any
 * pad can be routed to an SPI bus through the pin mux, and the chip select
 * is driven as a plain GPIO (software CS) so any pad may serve as CS.
 */

#define AMEBA_SPI0            0
#define AMEBA_SPI1            1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ameba_spi_register
 *
 * Description:
 *   Configure one Ameba SPI (DesignWare SSI) controller as a master bus,
 *   registering it with the NuttX SPI character driver at /dev/spiN, where N
 *   is the bus number.  The chip select is driven as a GPIO (software CS).
 *
 * Input Parameters:
 *   bus     - The controller index, AMEBA_SPI0 or AMEBA_SPI1.  Also used as
 *             the /dev/spiN minor number.
 *   clkpin  - The SCLK pad, encoded with AMEBA_PA()/AMEBA_PB().
 *   mosipin - The MOSI pad, encoded with AMEBA_PA()/AMEBA_PB().
 *   misopin - The MISO pad, encoded with AMEBA_PA()/AMEBA_PB().
 *   cspin   - The chip-select pad, driven as an active-low GPIO output.
 *
 * Returned Value:
 *   The SPI master lower half on success; NULL on failure.
 *
 ****************************************************************************/

struct spi_dev_s *ameba_spi_register(int bus, uint8_t clkpin,
                                     uint8_t mosipin, uint8_t misopin,
                                     uint8_t cspin);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_SPI_H */
