/****************************************************************************
 * arch/arm64/src/a64/a64_mipi_dsi.h
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

#ifndef __ARCH_ARM64_SRC_A64_A64_MIPI_DSI_H
#define __ARCH_ARM64_SRC_A64_A64_MIPI_DSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/a64_memorymap.h"
#include "mipi_dsi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Virtual Channel to be used for data transfer on the MIPI DSI Bus */

#define A64_MIPI_DSI_VIRTUAL_CHANNEL 0

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: a64_mipi_dsi_enable
 *
 * Description:
 *   Enable the MIPI DSI Block on the SoC. Should be called before
 *   transferring data on the MIPI DSI Bus.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_mipi_dsi_enable(void);

/****************************************************************************
 * Name: a64_mipi_dsi_write
 *
 * Description:
 *   Transmit the payload data to the MIPI DSI Bus as a MIPI DSI Short or
 *   Long Packet. This function is called to initialize the LCD Controller.
 *   Assumes that the MIPI DSI Block has been enabled on the SoC.
 *
 * Input Parameters:
 *   channel - Virtual Channel
 *   cmd     - DCS Command (Data Type)
 *   txbuf   - Payload data for the packet
 *   txlen   - Length of payload data (Max 65541 bytes)
 *
 * Returned Value:
 *   Number of bytes transmitted; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

ssize_t a64_mipi_dsi_write(uint8_t channel,
                           enum mipi_dsi_e cmd,
                           const uint8_t *txbuf,
                           size_t txlen);

/****************************************************************************
 * Name: a64_mipi_dsi_start
 *
 * Description:
 *   Start the MIPI DSI Bus in High Speed Clock Mode (HSC) for High Speed
 *   Data Transmission (HSD). Should be called after initializing the LCD
 *   Controller, and before executing any Display Engine operations.
 *   Assumes that the MIPI DSI Block has been enabled on the SoC.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_mipi_dsi_start(void);

#endif /* __ARCH_ARM64_SRC_A64_A64_MIPI_DSI_H */
