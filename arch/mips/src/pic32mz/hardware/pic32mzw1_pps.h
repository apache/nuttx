/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mzw1_pps.h
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

/* Peripheral Pin Select for PIC32MZ-W1/WFI32E01.
 *
 * Register addresses below come from the Microchip PIC32MZ-W_DFP
 * (Apache-2.0), proc/pwfi32e01.h.  The actual input/output function-select
 * VALUES (which 4-bit or 5-bit code picks which signal) are NOT in that
 * header - they were transcribed from the public PIC32MZ W1 and WFI32E01
 * Family Data Sheet (DS70005425), Section 13.4, Table 13-2 (input) and
 * Table 13-3 (output).
 *
 * This chip only implements PORTA, PORTB, PORTC and PORTK (see
 * hardware/pic32mz_ioport.h).
 *
 * Only the handful of functions needed for UART1/UART2 (the console/
 * debug UARTs) are defined so far - IC/OC/SPI/CAN/Ethernet PPS routing
 * is not yet covered.  Add groups following the same pattern as they are
 * needed.
 *
 * NOTE: UART1 also has dedicated, non-PPS pins (U1RX=RA8, U1TX=RA9).  A
 * board using those (e.g. ev49n51a) leaves BOARD_U1RX_PPS/
 * BOARD_U1TX_PPS undefined so that no PPS routing is programmed.
 */

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_PPS_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_PPS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "pic32mz_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PPS Register Addresses (input selectors) *********************************/

#define PIC32MZ_INT1R               (PIC32MZ_PPS_K1BASE + 0x0004)
#define PIC32MZ_INT2R               (PIC32MZ_PPS_K1BASE + 0x0008)
#define PIC32MZ_INT3R               (PIC32MZ_PPS_K1BASE + 0x000c)
#define PIC32MZ_INT4R               (PIC32MZ_PPS_K1BASE + 0x0010)
#define PIC32MZ_U1RXR               (PIC32MZ_PPS_K1BASE + 0x006c)
#define PIC32MZ_U1CTSR              (PIC32MZ_PPS_K1BASE + 0x0070)
#define PIC32MZ_U2RXR               (PIC32MZ_PPS_K1BASE + 0x0074)
#define PIC32MZ_U2CTSR              (PIC32MZ_PPS_K1BASE + 0x0078)
#define PIC32MZ_SDI1R               (PIC32MZ_PPS_K1BASE + 0x0098)
#define PIC32MZ_SDI2R               (PIC32MZ_PPS_K1BASE + 0x00a4)

/* PPS Register Addresses (output selectors, RPnR) - PORTA/PORTB/PORTK
 * pins only.
 */

#define PIC32MZ_RPA0R                (PIC32MZ_PPS_K1BASE + 0x0200)
#define PIC32MZ_RPA1R                (PIC32MZ_PPS_K1BASE + 0x0204)
#define PIC32MZ_RPA2R                (PIC32MZ_PPS_K1BASE + 0x0208)
#define PIC32MZ_RPA3R                (PIC32MZ_PPS_K1BASE + 0x020c)
#define PIC32MZ_RPA4R                (PIC32MZ_PPS_K1BASE + 0x0210)
#define PIC32MZ_RPA5R                (PIC32MZ_PPS_K1BASE + 0x0214)
#define PIC32MZ_RPA10R               (PIC32MZ_PPS_K1BASE + 0x0228)
#define PIC32MZ_RPA11R               (PIC32MZ_PPS_K1BASE + 0x022c)
#define PIC32MZ_RPA12R               (PIC32MZ_PPS_K1BASE + 0x0230)
#define PIC32MZ_RPA13R               (PIC32MZ_PPS_K1BASE + 0x0234)
#define PIC32MZ_RPA14R               (PIC32MZ_PPS_K1BASE + 0x0238)
#define PIC32MZ_RPA15R               (PIC32MZ_PPS_K1BASE + 0x023c)

#define PIC32MZ_RPB0R                (PIC32MZ_PPS_K1BASE + 0x0240)
#define PIC32MZ_RPB1R                (PIC32MZ_PPS_K1BASE + 0x0244)
#define PIC32MZ_RPB2R                (PIC32MZ_PPS_K1BASE + 0x0248)
#define PIC32MZ_RPB3R                (PIC32MZ_PPS_K1BASE + 0x024c)
#define PIC32MZ_RPB4R                (PIC32MZ_PPS_K1BASE + 0x0250)
#define PIC32MZ_RPB5R                (PIC32MZ_PPS_K1BASE + 0x0254)
#define PIC32MZ_RPB6R                (PIC32MZ_PPS_K1BASE + 0x0258)
#define PIC32MZ_RPB7R                (PIC32MZ_PPS_K1BASE + 0x025c)
#define PIC32MZ_RPB8R                (PIC32MZ_PPS_K1BASE + 0x0260)
#define PIC32MZ_RPB9R                (PIC32MZ_PPS_K1BASE + 0x0264)
#define PIC32MZ_RPB10R               (PIC32MZ_PPS_K1BASE + 0x0268)
#define PIC32MZ_RPB11R               (PIC32MZ_PPS_K1BASE + 0x026c)
#define PIC32MZ_RPB12R               (PIC32MZ_PPS_K1BASE + 0x0270)
#define PIC32MZ_RPB13R               (PIC32MZ_PPS_K1BASE + 0x0274)
#define PIC32MZ_RPB14R               (PIC32MZ_PPS_K1BASE + 0x0278)

#define PIC32MZ_RPK0R                (PIC32MZ_PPS_K1BASE + 0x02c0)
#define PIC32MZ_RPK1R                (PIC32MZ_PPS_K1BASE + 0x02c4)
#define PIC32MZ_RPK2R                (PIC32MZ_PPS_K1BASE + 0x02c8)
#define PIC32MZ_RPK3R                (PIC32MZ_PPS_K1BASE + 0x02cc)
#define PIC32MZ_RPK4R                (PIC32MZ_PPS_K1BASE + 0x02d0)
#define PIC32MZ_RPK5R                (PIC32MZ_PPS_K1BASE + 0x02d4)
#define PIC32MZ_RPK6R                (PIC32MZ_PPS_K1BASE + 0x02d8)
#define PIC32MZ_RPK7R                (PIC32MZ_PPS_K1BASE + 0x02dc)
#define PIC32MZ_RPK8R                (PIC32MZ_PPS_K1BASE + 0x02e0)
#define PIC32MZ_RPK9R                (PIC32MZ_PPS_K1BASE + 0x02e4)
#define PIC32MZ_RPK10R               (PIC32MZ_PPS_K1BASE + 0x02e8)
#define PIC32MZ_RPK11R               (PIC32MZ_PPS_K1BASE + 0x02ec)
#define PIC32MZ_RPK12R               (PIC32MZ_PPS_K1BASE + 0x02f0)
#define PIC32MZ_RPK13R               (PIC32MZ_PPS_K1BASE + 0x02f4)
#define PIC32MZ_RPK14R               (PIC32MZ_PPS_K1BASE + 0x02f8)

/* Input Pin Selection (Datasheet Table 13-2) *******************************
 *
 * U1RX shares the same 4-bit group as INT2/T3CK/T7CK/IC1/U2CTSn/C1RX/
 * ECRS/ERXD2/SS1/OCFB (restricted here to the A/B/K pins this chip has).
 */

#define U1RXR_RPA2                   0
#define U1RXR_RPA10                  1
#define U1RXR_RPA14                  2
#define U1RXR_RPB2                   3
#define U1RXR_RPB6                   4
#define U1RXR_RPB10                  5
#define U1RXR_RPB14                  6
#define U1RXR_RPK2                   11
#define U1RXR_RPK6                   12
#define U1RXR_RPK10                  12 /* NOTE: datasheet lists 1100 twice
                                          * (RPK6 and RPK10) - transcribed
                                          * as printed; verify against
                                          * silicon/errata before relying
                                          * on RPK10 specifically. */
#define U1RXR_RPK14                  13

/* U2RX shares the same 4-bit group as INT3/T2CK/T6CK/IC3/U1CTS/SDI2/
 * ERXD3/OCFC.
 */

#define U2RXR_RPA1                   0
#define U2RXR_RPA5                   1
#define U2RXR_RPA13                  2
#define U2RXR_RPB1                   3
#define U2RXR_RPB5                   4
#define U2RXR_RPB9                   5
#define U2RXR_RPB13                  6
#define U2RXR_RPK1                   11
#define U2RXR_RPK5                   12
#define U2RXR_RPK9                   12 /* NOTE: datasheet duplicate, see
                                          * U1RXR_RPK10 above. */
#define U2RXR_RPK13                  13

/* Output Pin Selection (Datasheet Table 13-3) ******************************
 *
 * U1TX is value 1 on the RPA0/RPA4/RPA12/RPB0/RPB4/RPB8/RPB12/RPK0/RPK4/
 * RPK8/RPK12 output group.
 */

#define U1TX_RPA0R                   1, PIC32MZ_RPA0R
#define U1TX_RPA4R                   1, PIC32MZ_RPA4R
#define U1TX_RPA12R                  1, PIC32MZ_RPA12R
#define U1TX_RPB0R                   1, PIC32MZ_RPB0R
#define U1TX_RPB4R                   1, PIC32MZ_RPB4R
#define U1TX_RPB8R                   1, PIC32MZ_RPB8R
#define U1TX_RPB12R                  1, PIC32MZ_RPB12R
#define U1TX_RPK0R                   1, PIC32MZ_RPK0R
#define U1TX_RPK4R                   1, PIC32MZ_RPK4R
#define U1TX_RPK8R                   1, PIC32MZ_RPK8R
#define U1TX_RPK12R                  1, PIC32MZ_RPK12R

/* U2TX is value 2 on the RPA3/RPA11/RPA15/RPB3/RPB7/RPB11/RPK3/RPK7/RPK11
 * output group.
 */

#define U2TX_RPA3R                   2, PIC32MZ_RPA3R
#define U2TX_RPA11R                  2, PIC32MZ_RPA11R
#define U2TX_RPA15R                  2, PIC32MZ_RPA15R
#define U2TX_RPB3R                   2, PIC32MZ_RPB3R
#define U2TX_RPB7R                   2, PIC32MZ_RPB7R
#define U2TX_RPB11R                  2, PIC32MZ_RPB11R
#define U2TX_RPK3R                   2, PIC32MZ_RPK3R
#define U2TX_RPK7R                   2, PIC32MZ_RPK7R
#define U2TX_RPK11R                  2, PIC32MZ_RPK11R

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZW1_PPS_H */
