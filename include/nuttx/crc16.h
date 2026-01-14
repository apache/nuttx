/****************************************************************************
 * include/nuttx/crc16.h
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

#ifndef __INCLUDE_NUTTX_CRC16_H
#define __INCLUDE_NUTTX_CRC16_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Append full suffix to avoid the penitential symbol collision */

#define crc16   crc16full

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
 * Name: crc16part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer.
 *
 *   The default polynomial is 0x1021 (x^16 + x^12 + x^5 + 1)
 *   See crc16xmodempart()
 *
 ****************************************************************************/

uint16_t crc16part(FAR const uint8_t *src, size_t len, uint16_t crc16val);

/****************************************************************************
 * Name: crc16
 *
 * Description:
 *   Return a 16-bit CRC of the contents of the 'src' buffer, length 'len'
 *
 *   The default polynomial is 0x1021 (x^16 + x^12 + x^5 + 1)
 *   See crc16xmodem()
 *
 ****************************************************************************/

uint16_t crc16(FAR const uint8_t *src, size_t len);

/****************************************************************************
 * Name: crc16ccittpart
 *
 * Description:
 *   Continue 16-bit CRC-CCITT calculation on a part of the buffer using the
 *   polynomial x^16+x^12+x^5+1.
 *
 *   This function is able to calculate any CRC that uses 0x1021 as it
 *   polynomial and requires reflecting both the input and the output.
 *   The following checksums can, among others, be calculated by this
 *   function, depending on the value provided for the initial seed and the
 *   value the final calculated CRC is XORed with:
 *
 *   - CRC-16/CCITT, CRC-16/CCITT-TRUE, CRC-16/KERMIT
 *   https://reveng.sourceforge.io/crc-catalogue/16.htm#crc.cat.crc-16-kermit
 *   initial seed: 0x0000, xor output: 0x0000
 *
 ****************************************************************************/

uint16_t crc16ccittpart(FAR const uint8_t *src, size_t len,
                        uint16_t crc16val);

/****************************************************************************
 * Name: crc16ccitt
 *
 * Description:
 *   Return a 16-bit CRC-CCITT of the contents of the 'src' buffer, length
 *   'len' using the polynomial x^16+x^12+x^5+1.
 *
 *   This function is able to calculate any CRC that uses 0x1021 as it
 *   polynomial and requires reflecting both the input and the output.
 *   The following checksums can, among others, be calculated by this
 *   function, depending on the value provided for the initial seed and the
 *   value the final calculated CRC is XORed with:
 *
 *   - CRC-16/CCITT, CRC-16/CCITT-TRUE, CRC-16/KERMIT
 *   https://reveng.sourceforge.io/crc-catalogue/16.htm#crc.cat.crc-16-kermit
 *   initial seed: 0x0000, xor output: 0x0000
 *
 ****************************************************************************/

uint16_t crc16ccitt(FAR const uint8_t *src, size_t len);

/****************************************************************************
 * Name: crc16ibm
 *
 * Description:
 *   Return a 16-bit CRC-ANSI of the contents of the 'src' buffer, length
 *   'len' using the polynomial 0x8005 (x^16 + x^15 + x^2 + 1).
 *
 *   The ANSI variant of CRC-16 uses 0x8005 (0xA001 reflected) as its
 *   polynomial with the initial * value set to 0x0000.
 *
 *   The following checksums can, among others, be calculated by this
 *   function, depending on the value provided for the initial seed and
 *   the value the final calculated CRC is XORed with:
 *
 *   - ARC, CRC-16, CRC-16/LHA, CRC-IBM
 *   https://reveng.sourceforge.io/crc-catalogue/16.htm#crc.cat.crc-16-arc
 *   poly: 0x8005 (0xA001) initial seed: 0x0000, xor output: 0x0000
 *
 ****************************************************************************/

uint16_t crc16ibmpart(FAR const uint8_t *src, size_t len,
                       uint16_t crc16val);

/****************************************************************************
 * Name: crc16ibm
 *
 * Description:
 *   Return a 16-bit CRC-ANSI of the contents of the 'src' buffer, length
 *   'len' using the polynomial 0x8005 (x^16 + x^15 + x^2 + 1).
 *
 *   The ANSI variant of CRC-16 uses 0x8005 (0xA001 reflected) as its
 *   polynomial with the initial * value set to 0x0000.
 *
 *   The following checksums can, among others, be calculated by this
 *   function, depending on the value provided for the initial seed and
 *   the value the final calculated CRC is XORed with:
 *
 *   - ARC, CRC-16, CRC-16/LHA, CRC-IBM
 *     https://reveng.sourceforge.io/crc-catalogue/16.htm#crc.cat.crc-16-arc
 *     poly: 0x8005 (0xA001) initial seed: 0x0000, xor output: 0x0000
 *
 ****************************************************************************/

uint16_t crc16ibm(FAR const uint8_t *src, size_t len);

/****************************************************************************
 * Name: crc16xmodempart
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer.
 *
 *   Return a 16-bit CRC-XMODEM of the contents of the 'src' buffer, length
 *   'len', using the polynomial 0x1021 (x^16 + x^12 + x^5 + 1).
 *
 *   The following checksums can, among others, be calculated by this
 *   function, depending on the value provided for the initial seed and
 *   the value the final calculated CRC is XORed with:
 *
 *   - Alias: CRC-16/ACORN, CRC-16/LTE, CRC-16/V-41-MSB, XMODEM, ZMODEM
 *   https://reveng.sourceforge.io/crc-catalogue/16.htm#crc.cat.crc-16-xmodem
 *   poly: 0x1021 initial seed: 0x0000, xor output: 0x0000
 *
 ****************************************************************************/

uint16_t crc16xmodempart(FAR const uint8_t *src, size_t len,
                         uint16_t crc16val);

/****************************************************************************
 * Name: crc16xmodem
 *
 * Description:
 *   Return a 16-bit CRC-XMODEM of the contents of the 'src' buffer, length
 *   'len', using the polynomial 0x1021 (x^16 + x^12 + x^5 + 1).
 *
 *   The following checksums can, among others, be calculated by this
 *   function, depending on the value provided for the initial seed and
 *   the value the final calculated CRC is XORed with:
 *
 *   - Alias: CRC-16/ACORN, CRC-16/LTE, CRC-16/V-41-MSB, XMODEM, ZMODEM
 *   https://reveng.sourceforge.io/crc-catalogue/16.htm#crc.cat.crc-16-xmodem
 *   poly: 0x1021 initial seed: 0x0000, xor output: 0x0000
 *
 ****************************************************************************/

uint16_t crc16xmodem(FAR const uint8_t *src, size_t len);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CRC16_H */
