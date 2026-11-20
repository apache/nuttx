/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_spi.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_SPI_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/spi/spi.h>
#ifdef CONFIG_SPI_SLAVE
#  include <nuttx/spi/slave.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RZV2H_SPI_CHANNELS  3
#define RZV2H_SPI_CHANNEL_0 0
#define RZV2H_SPI_CHANNEL_1 1
#define RZV2H_SPI_CHANNEL_2 2

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialize an enabled SPI_B master channel. */

struct spi_dev_s *rzv2h_spibus_initialize(int channel);

#ifdef CONFIG_SPI_SLAVE
/* Initialize an enabled SPI_B slave channel. */

struct spi_slave_ctrlr_s *rzv2h_spi_slave_initialize(int channel);
#endif

/* Master channels use one board-provided device-ID and chip-select
 * callback.  The board callback determines the channel from dev and applies
 * the selected 3-wire or 4-wire policy.
 */

#if defined(CONFIG_RZV2H_SPI_CHANNEL_0_MASTER) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_1_MASTER) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_2_MASTER)
void rzv2h_spiselect(struct spi_dev_s *dev, uint32_t devid,
                     bool selected);
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_0_MASTER
uint8_t rzv2h_spi0status(struct spi_dev_s *dev, uint32_t devid);
#  ifdef CONFIG_SPI_CMDDATA
int rzv2h_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#  endif
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_1_MASTER
uint8_t rzv2h_spi1status(struct spi_dev_s *dev, uint32_t devid);
#  ifdef CONFIG_SPI_CMDDATA
int rzv2h_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#  endif
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_2_MASTER
uint8_t rzv2h_spi2status(struct spi_dev_s *dev, uint32_t devid);
#  ifdef CONFIG_SPI_CMDDATA
int rzv2h_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#  endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_SPI_H */
