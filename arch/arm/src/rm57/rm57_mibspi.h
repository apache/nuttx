/****************************************************************************
 * arch/arm/src/rm57/rm57_mibspi.h
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

#ifndef __ARCH_ARM_SRC_RM57_RM57_MIBSPI_H
#define __ARCH_ARM_SRC_RM57_RM57_MIBSPI_H

/* SPI master driver for the five MibSPI modules (SPNU562A chapter 27).
 *
 * What is implemented: master mode on a single data line, in either of the
 * module's two modes, chosen per module in Kconfig:
 *
 * - compatibility mode: one word at a time through SPIDAT1/SPIBUF, polled,
 *   or by DMA on MibSPI1-3 for transfers above a threshold;
 * - multi-buffer mode: transfer group 0 over the buffer RAM, one pass of up
 *   to 128 words (255 on MibSPI1) per completion interrupt.
 *
 * What is not: DMA in multi-buffer mode, slave mode, the SPIENA handshake
 * and the parallel data lines of MibSPI5.
 *
 * Chip selects are never driven by the module.  Its automatic chip select
 * stays asserted, once held with CSHOLD, until the next control field is
 * loaded, so there is no clean way to release it at the moment the upper
 * half calls SPI_SELECT(false).  The SPICS pins are therefore left in GIO
 * mode and the board drives them from its select callback, either with the
 * helpers below or through any other output it prefers.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/spi/spi.h>

#include "hardware/rm57_mibspi.h"

#ifdef CONFIG_RM57_MIBSPI

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
 * Name: rm57_mibspibus_initialize
 *
 * Description:
 *   Bring one MibSPI module out of reset as an SPI master and return its
 *   lower half.  Later calls for the same bus return the same instance
 *   without touching the hardware again.
 *
 * Input Parameters:
 *   bus - Module number, 1-5
 *
 * Returned Value:
 *   The SPI device on success; NULL if the bus does not exist or is not
 *   enabled in the configuration.
 *
 ****************************************************************************/

struct spi_dev_s *rm57_mibspibus_initialize(int bus);

/****************************************************************************
 * Name: rm57_mibspi_csconfig
 *
 * Description:
 *   Turn the given SPICS pins of a module into GIO outputs, driven high
 *   (deselected).  The module is initialized first if it has not been.
 *
 * Input Parameters:
 *   bus    - Module number, 1-5
 *   csmask - One bit per SPICS pin, bit 0 for SPICS[0]
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rm57_mibspi_csconfig(int bus, uint8_t csmask);

/****************************************************************************
 * Name: rm57_mibspi_cswrite
 *
 * Description:
 *   Drive one SPICS pin that rm57_mibspi_csconfig() has claimed.  A single
 *   write to a set/clear register, so it is safe from any context.
 *
 * Input Parameters:
 *   bus   - Module number, 1-5
 *   cs    - SPICS pin number
 *   level - Pin level to drive
 *
 ****************************************************************************/

void rm57_mibspi_cswrite(int bus, int cs, bool level);

/****************************************************************************
 * Name: rm57_mibspiNselect, rm57_mibspiNstatus and rm57_mibspiNcmddata
 *
 * Description:
 *   Provided by board logic for every enabled bus; they become the select,
 *   status and cmddata methods of that bus's spi_ops_s.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_MIBSPI1
void rm57_mibspi1select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t rm57_mibspi1status(struct spi_dev_s *dev, uint32_t devid);
int rm57_mibspi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_RM57_MIBSPI2
void rm57_mibspi2select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t rm57_mibspi2status(struct spi_dev_s *dev, uint32_t devid);
int rm57_mibspi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_RM57_MIBSPI3
void rm57_mibspi3select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t rm57_mibspi3status(struct spi_dev_s *dev, uint32_t devid);
int rm57_mibspi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_RM57_MIBSPI4
void rm57_mibspi4select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t rm57_mibspi4status(struct spi_dev_s *dev, uint32_t devid);
int rm57_mibspi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_RM57_MIBSPI5
void rm57_mibspi5select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t rm57_mibspi5status(struct spi_dev_s *dev, uint32_t devid);
int rm57_mibspi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_RM57_MIBSPI */
#endif /* __ARCH_ARM_SRC_RM57_RM57_MIBSPI_H */
