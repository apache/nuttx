/****************************************************************************
 * arch/arm64/src/a64/mipi_dsi.c
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

/* Reference:
 *
 * "Understanding PinePhone's Display (MIPI DSI)"
 * https://lupyuen.github.io/articles/dsi
 *
 * "NuttX RTOS for PinePhone: Display Driver in Zig"
 * https://lupyuen.github.io/articles/dsi2
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/crc16.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include "mipi_dsi.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: compute_crc
 *
 * Description:
 *   Compute the MIPI DSI CRC for the data buffer.
 *
 * Input Parameters:
 *   data - Data buffer
 *   len  - Length of data buffer
 *
 * Returned Value:
 *   MIPI DSI CRC value of the data buffer
 *
 ****************************************************************************/

static uint16_t compute_crc(const uint8_t *data, size_t len)
{
  uint16_t crc;

  /* Compute  CRC-16-CCITT (x^16+x^12+x^5+1) */

  DEBUGASSERT(data != NULL);
  crc = crc16ccittpart(data, len, 0xffff);

  return crc;
}

/****************************************************************************
 * Name: compute_ecc
 *
 * Description:
 *   Compute the MIPI DSI Error Correction Code (ECC) for the 3-byte
 *   Packet Header. The ECC allows single-bit errors to be corrected and
 *   2-bit errors to be detected in the MIPI DSI Packet Header.
 *
 * Input Parameters:
 *   di_wc - Packet Header (Data Identifier + Word Count)
 *   len   - Length of Packet Header (Should be 3 bytes)
 *
 * Returned Value:
 *   MIPI DSI ECC value of the Packet Header
 *
 ****************************************************************************/

static uint8_t compute_ecc(const uint8_t *di_wc, size_t len)
{
  uint32_t di_wc_word;
  bool d[24];
  bool ecc[8];
  int i;

  /* Packet Header should be exactly 3 bytes */

  DEBUGASSERT(di_wc != NULL);
  if (len != 3)
    {
      DEBUGPANIC();
      return 0;
    }

  /* Combine Data Identifier and Word Count into a 24-bit word */

  di_wc_word = di_wc[0] | (di_wc[1] << 8) | (di_wc[2] << 16);

  /* Extract the 24 bits from the word */

  memset(d, 0, sizeof(d));
  for (i = 0; i < 24; i++)
    {
      d[i] = di_wc_word & 1;
      di_wc_word >>= 1;
    }

  /* Compute the ECC bits */

  memset(ecc, 0, sizeof(ecc));
  ecc[7] = 0;
  ecc[6] = 0;
  ecc[5] = d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^
           d[18] ^ d[19] ^ d[21] ^ d[22] ^ d[23];
  ecc[4] = d[4]  ^ d[5]  ^ d[6]  ^ d[7]  ^ d[8]  ^ d[9]  ^ d[16] ^ d[17] ^
           d[18] ^ d[19] ^ d[20] ^ d[22] ^ d[23];
  ecc[3] = d[1]  ^ d[2]  ^ d[3]  ^ d[7]  ^ d[8]  ^ d[9]  ^ d[13] ^ d[14] ^
           d[15] ^ d[19] ^ d[20] ^ d[21] ^ d[23];
  ecc[2] = d[0]  ^ d[2]  ^ d[3]  ^ d[5]  ^ d[6]  ^ d[9]  ^ d[11] ^ d[12] ^
           d[15] ^ d[18] ^ d[20] ^ d[21] ^ d[22];
  ecc[1] = d[0]  ^ d[1]  ^ d[3]  ^ d[4]  ^ d[6]  ^ d[8]  ^ d[10] ^ d[12] ^
           d[14] ^ d[17] ^ d[20] ^ d[21] ^ d[22] ^ d[23];
  ecc[0] = d[0]  ^ d[1]  ^ d[2]  ^ d[4]  ^ d[5]  ^ d[7]  ^ d[10] ^ d[11] ^
           d[13] ^ d[16] ^ d[20] ^ d[21] ^ d[22] ^ d[23];

  /* Merge the ECC bits */

  return ecc[0] | (ecc[1] << 1) | (ecc[2] << 2) | (ecc[3] << 3) |
         (ecc[4] << 4) | (ecc[5] << 5) | (ecc[6] << 6) | (ecc[7] << 7);
}

/****************************************************************************
 * Public Functions
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
                             size_t txlen)
{
  /* Data Identifier (DI) (1 byte):
   * Virtual Channel Identifier (Bits 6 to 7)
   * Data Type (Bits 0 to 5)
   */

  const uint8_t vc = channel;
  const uint8_t dt = cmd;
  const uint8_t di = (vc << 6) | dt;

  /* Word Count (WC) (2 bytes)：
   * Number of bytes in the Packet Payload
   */

  const uint16_t wc = txlen;
  const uint8_t wcl = wc & 0xff;
  const uint8_t wch = wc >> 8;

  /* Data Identifier + Word Count (3 bytes):
   * For computing Error Correction Code (ECC)
   */

  const uint8_t di_wc[3] =
    {
      di,
      wcl,
      wch
    };

  /* Compute ECC for Data Identifier + Word Count */

  const uint8_t ecc = compute_ecc(di_wc, sizeof(di_wc));

  /* Packet Header (4 bytes):
   * Data Identifier + Word Count + Error Correction Code
   */

  const uint8_t header[4] =
    {
      di_wc[0],
      di_wc[1],
      di_wc[2],
      ecc
    };

  /* Checksum (CS) (2 bytes):
   * 16-bit Cyclic Redundancy Check (CRC) of the Payload
   * (Not the entire packet)
   */

  const uint16_t cs = compute_crc(txbuf, txlen);
  const uint8_t csl = cs & 0xff;
  const uint8_t csh = cs >> 8;

  /* Packet Footer (2 bytes):
   * Checksum (CS)
   */

  const uint8_t footer[2] =
    {
      csl,
      csh
    };

  /* Packet Length:
   * Packet Header (4 bytes)
   * Payload (txlen bytes)
   * Packet Footer (2 bytes)
   */

  const size_t len = sizeof(header) + txlen + sizeof(footer);

  ginfo("channel=%d, cmd=0x%x, txlen=%ld\n", channel, cmd, txlen);
  DEBUGASSERT(pktbuf != NULL && txbuf != NULL);
  DEBUGASSERT(channel < 4);
  DEBUGASSERT(cmd < (1 << 6));

  if (txlen > 65541)  /* Max 65,541 bytes for payload */
    {
      DEBUGPANIC();
      return ERROR;
    }

  if (len > pktlen)  /* Packet Buffer too small */
    {
      DEBUGPANIC();
      return ERROR;
    }

  /* Copy Packet Header, Payload, Packet Footer to Packet Buffer */

  memcpy(pktbuf, header, sizeof(header));
  memcpy(pktbuf + sizeof(header), txbuf, txlen);
  memcpy(pktbuf + sizeof(header) + txlen, footer, sizeof(footer));

  /* Return the Packet Length */

  return len;
}

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
                              size_t txlen)
{
  /* A Short Packet consists of 8-bit Data Identifier (DI),
   * 2 bytes of commands or data, and 8-bit ECC.
   * The length of a short packet is 4 bytes including ECC.
   * Thus a MIPI DSI Short Packet (compared with Long Packet):
   * - Doesn't have Packet Payload and Packet Footer (CRC)
   * - Instead of Word Count (WC), the Packet Header now has 2 bytes of data
   * Everything else is the same.
   */

  /* Data Identifier (DI) (1 byte):
   * Virtual Channel Identifier (Bits 6 to 7)
   * Data Type (Bits 0 to 5)
   */

  const uint8_t vc = channel;
  const uint8_t dt = cmd;
  const uint8_t di = (vc << 6) | dt;

  /* Data (2 bytes): Fill with 0 if Second Byte is missing */

  const uint8_t data[2] =
    {
      txbuf[0],                     /* First Byte */
      (txlen == 2) ? txbuf[1] : 0,  /* Second Byte */
    };

  /* Data Identifier + Data (3 bytes):
   * For computing Error Correction Code (ECC)
   */

  const uint8_t di_data[3] =
    {
      di,
      data[0],
      data[1]
    };

  /* Compute ECC for Data Identifier + Word Count */

  const uint8_t ecc = compute_ecc(di_data, sizeof(di_data));

  /* Packet Header (4 bytes):
   * Data Identifier + Data + Error Correction Code
   */

  const uint8_t header[4] =
    {
      di_data[0],
      di_data[1],
      di_data[2],
      ecc
    };

  /* Packet Length is Packet Header Size (4 bytes) */

  const size_t len = sizeof(header);

  ginfo("channel=%d, cmd=0x%x, txlen=%ld\n", channel, cmd, txlen);
  DEBUGASSERT(pktbuf != NULL && txbuf != NULL);
  DEBUGASSERT(channel < 4);
  DEBUGASSERT(cmd < (1 << 6));

  if (txlen < 1 || txlen > 2)  /* Payload should be 1 or 2 bytes */
    {
      DEBUGPANIC();
      return ERROR;
    }

  if (len > pktlen)  /* Packet Buffer too small */
    {
      DEBUGPANIC();
      return ERROR;
    }

  /* Copy Packet Header to Packet Buffer */

  memcpy(pktbuf, header, sizeof(header));  /* 4 bytes */

  /* Return the Packet Length */

  return len;
}
