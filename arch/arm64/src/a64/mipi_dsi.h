/****************************************************************************
 * arch/arm64/src/a64/mipi_dsi.h
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

#ifndef __ARCH_ARM64_SRC_A64_MIPI_DSI_H
#define __ARCH_ARM64_SRC_A64_MIPI_DSI_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MIPI DSI Processor-to-Peripheral Transaction Types */

enum mipi_dsi_e
{
  /* DCS Short Write (Without Parameter) */

  MIPI_DSI_DCS_SHORT_WRITE       = 0x05,

  /* DCS Short Write (With Parameter) */

  MIPI_DSI_DCS_SHORT_WRITE_PARAM = 0x15,

  /* DCS Long Write */

  MIPI_DSI_DCS_LONG_WRITE        = 0x39
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mipi_dsi_long_packet
 *
 * Description:
 *   Compose a MIPI DSI Long Packet. A Short Packet consists of Data
 *   Identifier (Virtual Channel + Data Type), Word Count (Payload Size),
 *   Error Correction Code, Payload and Checksum. Packet Length is
 *   Payload Size + 6 bytes.
 *
 * Input Parameters:
 *   pktbuf  - Buffer for the returned packet
 *   pktlen  - Size of the packet buffer
 *   channel - Virtual Channel
 *   cmd     - DCS Command (Data Type)
 *   txbuf   - Payload data for the packet
 *   txlen   - Length of payload data (Max 65541 bytes)
 *
 * Returned Value:
 *   Number of bytes in the returned packet; ERROR (-1) if packet buffer is
 *   too small for the packet
 *
 ****************************************************************************/

ssize_t mipi_dsi_long_packet(uint8_t *pktbuf,
                             size_t pktlen,
                             uint8_t channel,
                             enum mipi_dsi_e cmd,
                             const uint8_t *txbuf,
                             size_t txlen);

/****************************************************************************
 * Name: mipi_dsi_short_packet
 *
 * Description:
 *   Compose a MIPI DSI Short Packet. A Short Packet consists of Data
 *   Identifier (Virtual Channel + Data Type), Data (1 or 2 bytes) and
 *   Error Correction Code. Packet Length is 4 bytes.
 *
 * Input Parameters:
 *   pktbuf  - Buffer for the returned packet
 *   pktlen  - Size of the packet buffer
 *   channel - Virtual Channel
 *   cmd     - DCS Command (Data Type)
 *   txbuf   - Payload data for the packet
 *   txlen   - Length of payload data (1 or 2 bytes)
 *
 * Returned Value:
 *   Number of bytes in the returned packet; ERROR (-1) if packet buffer is
 *   too small for the packet
 *
 ****************************************************************************/

ssize_t mipi_dsi_short_packet(uint8_t *pktbuf,
                              size_t pktlen,
                              uint8_t channel,
                              enum mipi_dsi_e cmd,
                              const uint8_t *txbuf,
                              size_t txlen);

#endif /* __ARCH_ARM64_SRC_A64_MIPI_DSI_H */
