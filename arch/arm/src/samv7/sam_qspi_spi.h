/****************************************************************************
 * arch/arm/src/samv7/sam_qspi_spi.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_QSPI_SPI_H
#define __ARCH_ARM_SRC_SAMV7_SAM_QSPI_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "sam_config.h"

#ifdef CONFIG_SAMV7_QSPI_SPI_MODE

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
 * Name: sam_qspi_spi_initialize
 *
 * Description:
 *   Initialize the selected QSPI port in SPI master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s;
FAR struct spi_dev_s *sam_qspi_spi_initialize(int intf);

/****************************************************************************
 * Name: sam_qspi_select
 *
 * Description:
 *   PIO chip select pins may be programmed by the board specific logic in
 *   one of two different ways.  First, the pins may be programmed as SPI
 *   peripherals.  In that case, the pins are completely controlled by the
 *   SPI driver.  This method still needs to be provided, but it may be only
 *   a stub.
 *
 *   An alternative way to program the PIO chip select pins is as a normal
 *   PIO output.  In that case, the automatic control of the CS pins is
 *   bypassed and this function must provide control of the chip select.
 *   NOTE:  In this case, the PIO output pin does *not* have to be the
 *   same as the NPCS pin normal associated with the chip select number.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *   selected - TRUE:Select the device, FALSE:De-select the device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_qspi_select(uint32_t devid, bool selected);

/****************************************************************************
 * Name: sam_qspi_status
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *
 * Returned Value:
 *   Bit-encoded SPI status (see include/nuttx/spi/spi.h.
 *
 ****************************************************************************/

uint8_t sam_qspi_status(FAR struct spi_dev_s *dev, uint32_t devid);

/****************************************************************************
 * Name: sam_qspi_cmddata
 *
 * Description:
 *   Some SPI devices require an additional control to determine if the SPI
 *   data being sent is a command or is data.  If CONFIG_SPI_CMDDATA then
 *   this function will be called to different be command and data transfers.
 *
 *   This is often needed, for example, by LCD drivers.  Some LCD hardware
 *   may be configured to use 9-bit data transfers with the 9th bit
 *   indicating command or data.  That same hardware may be configurable,
 *   instead, to use 8-bit data but to require an additional, board-
 *   specific PIO control to distinguish command and data.  This function
 *   would be needed in that latter case.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
int sam_qspi_cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif /* CONFIG_SPI_CMDDATA */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SAMV7_QSPI_SPI_MODE */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_QSPI_SPI_H */
