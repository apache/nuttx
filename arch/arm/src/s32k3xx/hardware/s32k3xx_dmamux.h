/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_dmamux.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_DMAMUX_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMAMUX Register Offsets **************************************************/

#define S32K3XX_DMAMUX_CHCFG3_OFFSET  (0x00) /* Channel Configuration Register 3 (CHCFG3) */
#define S32K3XX_DMAMUX_CHCFG2_OFFSET  (0x01) /* Channel Configuration Register 2 (CHCFG2) */
#define S32K3XX_DMAMUX_CHCFG1_OFFSET  (0x02) /* Channel Configuration Register 1 (CHCFG1) */
#define S32K3XX_DMAMUX_CHCFG0_OFFSET  (0x03) /* Channel Configuration Register 0 (CHCFG0) */
#define S32K3XX_DMAMUX_CHCFG7_OFFSET  (0x04) /* Channel Configuration Register 7 (CHCFG7) */
#define S32K3XX_DMAMUX_CHCFG6_OFFSET  (0x05) /* Channel Configuration Register 6 (CHCFG6) */
#define S32K3XX_DMAMUX_CHCFG5_OFFSET  (0x06) /* Channel Configuration Register 5 (CHCFG5) */
#define S32K3XX_DMAMUX_CHCFG4_OFFSET  (0x07) /* Channel Configuration Register 4 (CHCFG4) */
#define S32K3XX_DMAMUX_CHCFG11_OFFSET (0x08) /* Channel Configuration Register 11 (CHCFG11) */
#define S32K3XX_DMAMUX_CHCFG10_OFFSET (0x09) /* Channel Configuration Register 10 (CHCFG10) */
#define S32K3XX_DMAMUX_CHCFG9_OFFSET  (0x0a) /* Channel Configuration Register 9 (CHCFG9) */
#define S32K3XX_DMAMUX_CHCFG8_OFFSET  (0x0b) /* Channel Configuration Register 8 (CHCFG8) */
#define S32K3XX_DMAMUX_CHCFG15_OFFSET (0x0c) /* Channel Configuration Register 15 (CHCFG15) */
#define S32K3XX_DMAMUX_CHCFG14_OFFSET (0x0d) /* Channel Configuration Register 14 (CHCFG14) */
#define S32K3XX_DMAMUX_CHCFG13_OFFSET (0x0e) /* Channel Configuration Register 13 (CHCFG13) */
#define S32K3XX_DMAMUX_CHCFG12_OFFSET (0x0f) /* Channel Configuration Register 12 (CHCFG12) */

#define S32K3XX_DMAMUX_CHCFG_OFFSET(n) ((n) + 3 - 2 * ((n) % 4))

#define S32K3XX_DMAMUX0_CHCFG(n)        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG_OFFSET(n))
#define S32K3XX_DMAMUX1_CHCFG(n)        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG_OFFSET(n))

/* DMAMUX Register Addresses ************************************************/

#define S32K3XX_DMAMUX0_CHCFG3        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG3_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG2        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG2_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG1        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG1_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG0        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG0_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG7        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG7_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG6        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG6_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG5        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG5_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG4        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG4_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG11       (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG11_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG10       (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG10_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG9        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG9_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG8        (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG8_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG15       (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG15_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG14       (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG14_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG13       (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG13_OFFSET)
#define S32K3XX_DMAMUX0_CHCFG12       (S32K3XX_DMAMUX0_BASE + S32K3XX_DMAMUX_CHCFG12_OFFSET)

#define S32K3XX_DMAMUX1_CHCFG3        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG3_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG2        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG2_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG1        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG1_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG0        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG0_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG7        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG7_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG6        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG6_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG5        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG5_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG4        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG4_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG11       (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG11_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG10       (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG10_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG9        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG9_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG8        (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG8_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG15       (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG15_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG14       (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG14_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG13       (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG13_OFFSET)
#define S32K3XX_DMAMUX1_CHCFG12       (S32K3XX_DMAMUX1_BASE + S32K3XX_DMAMUX_CHCFG12_OFFSET)

/* DMAMUX Register Bitfield Definitions *************************************/

#define DMAMUX_CHCFG_SOURCE_SHIFT     (0)      /* Bits 0-5: DMA Channel Source (SOURCE) */
#define DMAMUX_CHCFG_SOURCE_MASK      (0x3f << DMAMUX_CHCFG_SOURCE_SHIFT)
#define DMAMUX_CHCFG_TRIG             (1 << 6) /* Bit 6: DMA Channel Trigger Enable (TRIG) */
#define DMAMUX_CHCFG_ENBL             (1 << 7) /* Bit 7: DMA Channel Enable (ENBL) */
#define DMAMUX_CHCFG_MASK             (0xff)   /* Bits 0-7 */

#define DMAMUX_CHCFG_DMAMUX1          (1 << 15) /* Bit 15: DMAMUX1 Selection */

/* DMA Request sources */

/** edma_mux0 **/

#define DMA_REQ_DISABLED0    (0)  ///< Channel disabled (default)
#define DMA_REQ_SIUL_0       (1)  ///< SIUL DMA request 0
#define DMA_REQ_SIUL_1       (2)  ///< SIUL DMA request 1
#define DMA_REQ_SIUL_2       (3)  ///< SIUL DMA request 2
#define DMA_REQ_SIUL_3       (4)  ///< SIUL DMA request 3
#define DMA_REQ_SIUL_4       (5)  ///< SIUL DMA request 4
#define DMA_REQ_SIUL_5       (6)  ///< SIUL DMA request 5
#define DMA_REQ_SIUL_6       (7)  ///< SIUL DMA request 6
#define DMA_REQ_SIUL_7       (8)  ///< SIUL DMA request 7
#define DMA_REQ_BCTU_FIFO1   (10) ///< BCTU DMA FIFO1 request
#define DMA_REQ_BCTU_0       (10) ///< BCTU DMA request 0
#define DMA_REQ_BCTU_1       (11) ///< BCTU DMA request 1
#define DMA_REQ_EMIOS0_0     (12) ///< eMIOS0 DMA request ch0
#define DMA_REQ_EMIOS0_1     (13) ///< eMIOS0 DMA request ch1
#define DMA_REQ_EMIOS0_9     (14) ///< eMIOS0 DMA request ch9
#define DMA_REQ_EMIOS0_10    (15) ///< eMIOS0 DMA request ch10
#define DMA_REQ_EMIOS1_0     (16) ///< eMIOS1 DMA request ch0
#define DMA_REQ_EMIOS1_1     (17) ///< eMIOS1 DMA request ch1
#define DMA_REQ_EMIOS1_9     (18) ///< eMIOS1 DMA request ch9
#define DMA_REQ_EMIOS1_10    (19) ///< eMIOS1 DMA request ch10
#define DMA_REQ_EMIOS2_0     (20) ///< eMIOS2 DMA request ch0
#define DMA_REQ_EMIOS2_1     (21) ///< eMIOS2 DMA request ch1
#define DMA_REQ_EMIOS2_9     (22) ///< eMIOS2 DMA request ch9
#define DMA_REQ_EMIOS2_10    (23) ///< eMIOS2 DMA request ch10
#define DMA_REQ_LCU0_0       (24) ///< LCU0 DMA request 0
#define DMA_REQ_LCU1_0       (25) ///< LCU1 DMA request 0
#define DMA_REQ_RESERVED1    (26) ///< RESERVED
#define DMA_REQ_RESERVED2    (27) ///< RESERVED
#define DMA_REQ_RESERVED3    (28) ///< RESERVED
#define DMA_REQ_FLEXCAN0     (29) ///< FLEXCAN0 DMA request
#define DMA_REQ_FLEXCAN1     (30) ///< FLEXCAN1 DMA request
#define DMA_REQ_FLEXCAN2     (31) ///< FLEXCAN2 DMA request
#define DMA_REQ_FLEXCAN3     (32) ///< FLEXCAN3 DMA request
#define DMA_REQ_FLEXIO_0     (33) ///< FLEXIO DMA shifter0 | timer0 request
#define DMA_REQ_FLEXIO_1     (34) ///< FLEXIO DMA shifter1 | timer1 request
#define DMA_REQ_FLEXIO_2     (35) ///< FLEXIO DMA shifter2 | timer2 request
#define DMA_REQ_FLEXIO_3     (36) ///< FLEXIO DMA shifter3 | timer3 request
#define DMA_REQ_LPUART08_TX  (37) ///< LPUART0 | LPUART8 DMA transmit request
#define DMA_REQ_LPUART08_RX  (38) ///< LPUART0 | LPUART8 DMA receive request
#define DMA_REQ_LPUART19_TX  (39) ///< LPUART1 | LPUART9 DMA transmit request
#define DMA_REQ_LPUART19_RX  (40) ///< LPUART1 | LPUART9 DMA receive request
#define DMA_REQ_LPI2C0_RX    (41) ///< LPI2C0 DMA receive | receive slave request
#define DMA_REQ_LPI2C0_TX    (42) ///< LPI2C0 DMA transmit | transmit slave request
#define DMA_REQ_LPSPI0_TX    (43) ///< LPSPI0 DMA transmit request
#define DMA_REQ_LPSPI0_RX    (44) ///< LPSPI0 DMA receive request
#define DMA_REQ_LPSPI1_TX    (45) ///< LPSPI1 DMA transmit request
#define DMA_REQ_LPSPI1_RX    (46) ///< LPSPI1 DMA receive request
#define DMA_REQ_LPSPI2_TX    (47) ///< LPSPI2 DMA transmit request
#define DMA_REQ_LPSPI2_RX    (48) ///< LPSPI2 DMA receive request
#define DMA_REQ_LPSPI3_TX    (49) ///< LPSPI3 DMA transmit request
#define DMA_REQ_LPSPI3_RX    (50) ///< LPSPI3 DMA receive request
#define DMA_REQ_I3C0_RX      (51) ///< I3C0 DMA receive request
#define DMA_REQ_I3C0_TX      (52) ///< I3C0 DMA transmit request
#define DMA_REQ_QSPI_RX      (53) ///< QSPI DMA receive buffer drain request
#define DMA_REQ_QSPI_TX      (54) ///< QSPI DMA transmit buffer fill request
#define DMA_REQ_SAI0_RX      (55) ///< SAI0 DMA receive request
#define DMA_REQ_SAI0_TX      (56) ///< SAI0 DMA transmit request
#define DMA_REQ_RESERVED4    (57) ///< RESERVED
#define DMA_REQ_ADC0         (58) ///< ADC0 DMA request
#define DMA_REQ_ADC1         (59) ///< ADC1 DMA request
#define DMA_REQ_ADC2         (60) ///< ADC2 DMA request
#define DMA_REQ_LPCMP0       (61) ///< LPCMP0 DMA request
#define DMA_REQ_ENABLED0     (62) ///< Always enabled
#define DMA_REQ_ENABLED1     (63) ///< Always enabled                   */

/** edma_mux1 **/

#define DMA_REQ_DISABLED1    (DMAMUX_CHCFG_DMAMUX1 | 0)  ///< Channel disabled (default)
#define DMA_REQ_SIUL_8       (DMAMUX_CHCFG_DMAMUX1 | 1)  ///< SIUL DMA request 8
#define DMA_REQ_SIUL_9       (DMAMUX_CHCFG_DMAMUX1 | 2)  ///< SIUL DMA request 9
#define DMA_REQ_SIUL_10      (DMAMUX_CHCFG_DMAMUX1 | 3)  ///< SIUL DMA request 10
#define DMA_REQ_SIUL_11      (DMAMUX_CHCFG_DMAMUX1 | 4)  ///< SIUL DMA request 11
#define DMA_REQ_SIUL_12      (DMAMUX_CHCFG_DMAMUX1 | 5)  ///< SIUL DMA request 12
#define DMA_REQ_SIUL_13      (DMAMUX_CHCFG_DMAMUX1 | 6)  ///< SIUL DMA request 13
#define DMA_REQ_SIUL_14      (DMAMUX_CHCFG_DMAMUX1 | 7)  ///< SIUL DMA request 14
#define DMA_REQ_SIUL_15      (DMAMUX_CHCFG_DMAMUX1 | 8)  ///< SIUL DMA request 15
#define DMA_REQ_BCTU_FIFO2   (DMAMUX_CHCFG_DMAMUX1 | 9)  ///< BCTU DMA FIFO2 request
#define DMA_REQ_BCTU_2       (DMAMUX_CHCFG_DMAMUX1 | 10) ///< BCTU DMA request 2
#define DMA_REQ_EMIOS0_16    (DMAMUX_CHCFG_DMAMUX1 | 11) ///< eMIOS0 DMA request ch16
#define DMA_REQ_EMIOS0_17    (DMAMUX_CHCFG_DMAMUX1 | 12) ///< eMIOS0 DMA request ch17
#define DMA_REQ_EMIOS0_18    (DMAMUX_CHCFG_DMAMUX1 | 13) ///< eMIOS0 DMA request ch18
#define DMA_REQ_EMIOS0_19    (DMAMUX_CHCFG_DMAMUX1 | 14) ///< eMIOS0 DMA request ch19
#define DMA_REQ_EMIOS1_16    (DMAMUX_CHCFG_DMAMUX1 | 15) ///< eMIOS1 DMA request ch16
#define DMA_REQ_EMIOS1_17    (DMAMUX_CHCFG_DMAMUX1 | 16) ///< eMIOS1 DMA request ch17
#define DMA_REQ_EMIOS1_18    (DMAMUX_CHCFG_DMAMUX1 | 17) ///< eMIOS1 DMA request ch18
#define DMA_REQ_EMIOS1_19    (DMAMUX_CHCFG_DMAMUX1 | 18) ///< eMIOS1 DMA request ch19
#define DMA_REQ_EMIOS2_16    (DMAMUX_CHCFG_DMAMUX1 | 19) ///< eMIOS2 DMA request ch16
#define DMA_REQ_EMIOS2_17    (DMAMUX_CHCFG_DMAMUX1 | 20) ///< eMIOS2 DMA request ch17
#define DMA_REQ_EMIOS2_18    (DMAMUX_CHCFG_DMAMUX1 | 21) ///< eMIOS2 DMA request ch18
#define DMA_REQ_EMIOS2_19    (DMAMUX_CHCFG_DMAMUX1 | 22) ///< eMIOS2 DMA request ch19
#define DMA_REQ_LCU0_1       (DMAMUX_CHCFG_DMAMUX1 | 23) ///< LCU0 DMA request 1
#define DMA_REQ_LCU0_2       (DMAMUX_CHCFG_DMAMUX1 | 24) ///< LCU1 DMA request 2
#define DMA_REQ_LCU1_1       (DMAMUX_CHCFG_DMAMUX1 | 25) ///< LCU1 DMA request 1
#define DMA_REQ_LCU1_2       (DMAMUX_CHCFG_DMAMUX1 | 26) ///< LCU1 DMA request 2
#define DMA_REQ_ENET_0       (DMAMUX_CHCFG_DMAMUX1 | 27) ///< ENET IEEE 1588 PTP timer ch[0] DMA request
#define DMA_REQ_ENET_1       (DMAMUX_CHCFG_DMAMUX1 | 27) ///< ENET IEEE 1588 PTP timer ch[1] DMA request
#define DMA_REQ_ENET_2       (DMAMUX_CHCFG_DMAMUX1 | 27) ///< ENET IEEE 1588 PTP timer ch[2] DMA request
#define DMA_REQ_ENET_3       (DMAMUX_CHCFG_DMAMUX1 | 27) ///< ENET IEEE 1588 PTP timer ch[3] DMA request
#define DMA_REQ_RESERVED5    (DMAMUX_CHCFG_DMAMUX1 | 28) ///< RESERVED
#define DMA_REQ_RESERVED6    (DMAMUX_CHCFG_DMAMUX1 | 29) ///< RESERVED
#define DMA_REQ_FLECAN4      (DMAMUX_CHCFG_DMAMUX1 | 30) ///< FLEXCAN4 DMA request
#define DMA_REQ_FLECAN5      (DMAMUX_CHCFG_DMAMUX1 | 31) ///< FLEXCAN5 DMA request
#define DMA_REQ_RESERVED7    (DMAMUX_CHCFG_DMAMUX1 | 32) ///< RESERVED
#define DMA_REQ_RESERVED8    (DMAMUX_CHCFG_DMAMUX1 | 33) ///< RESERVED
#define DMA_REQ_FLEXIO_4     (DMAMUX_CHCFG_DMAMUX1 | 34) ///< FLEXIO DMA shifter4 | timer4 request
#define DMA_REQ_FLEXIO_5     (DMAMUX_CHCFG_DMAMUX1 | 35) ///< FLEXIO DMA shifter5 | timer5 request
#define DMA_REQ_FLEXIO_6     (DMAMUX_CHCFG_DMAMUX1 | 36) ///< FLEXIO DMA shifter6 | timer6 request
#define DMA_REQ_FLEXIO_7     (DMAMUX_CHCFG_DMAMUX1 | 37) ///< FLEXIO DMA shifter7 | timer7 request
#define DMA_REQ_LPUART210_TX (DMAMUX_CHCFG_DMAMUX1 | 38) ///< LPUART2 | LPUART10 DMA transmit request
#define DMA_REQ_LPUART210_RX (DMAMUX_CHCFG_DMAMUX1 | 39) ///< LPUART2 | LPUART10 DMA receive request
#define DMA_REQ_LPUART311_TX (DMAMUX_CHCFG_DMAMUX1 | 40) ///< LPUART3 | LPUART11 DMA transmit request
#define DMA_REQ_LPUART311_RX (DMAMUX_CHCFG_DMAMUX1 | 41) ///< LPUART3 | LPUART11 DMA receive request
#define DMA_REQ_LPUART412_TX (DMAMUX_CHCFG_DMAMUX1 | 42) ///< LPUART4 | LPUART12 DMA transmit request
#define DMA_REQ_LPUART412_RX (DMAMUX_CHCFG_DMAMUX1 | 43) ///< LPUART4 | LPUART12 DMA receive request
#define DMA_REQ_LPUART513_TX (DMAMUX_CHCFG_DMAMUX1 | 44) ///< LPUART5 | LPUART13 DMA transmit request
#define DMA_REQ_LPUART513_RX (DMAMUX_CHCFG_DMAMUX1 | 45) ///< LPUART5 | LPUART13 DMA receive request
#define DMA_REQ_LPUART614_TX (DMAMUX_CHCFG_DMAMUX1 | 46) ///< LPUART6 | LPUART14 DMA transmit request
#define DMA_REQ_LPUART614_RX (DMAMUX_CHCFG_DMAMUX1 | 47) ///< LPUART6 | LPUART14 DMA receive request
#define DMA_REQ_LPUART715_TX (DMAMUX_CHCFG_DMAMUX1 | 48) ///< LPUART7 | LPUART15 DMA transmit request
#define DMA_REQ_LPUART715_RX (DMAMUX_CHCFG_DMAMUX1 | 49) ///< LPUART7 | LPUART15 DMA receive request
#define DMA_REQ_LPI2C1_RX    (DMAMUX_CHCFG_DMAMUX1 | 50) ///< LPI2C1 DMA receive | receive slave request
#define DMA_REQ_LPI2C1_TX    (DMAMUX_CHCFG_DMAMUX1 | 51) ///< LPI2C1 DMA transmit | transmit slave request
#define DMA_REQ_LPSPI4_TX    (DMAMUX_CHCFG_DMAMUX1 | 52) ///< LPSPI4 DMA transmit request
#define DMA_REQ_LPSPI4_RX    (DMAMUX_CHCFG_DMAMUX1 | 53) ///< LPSPI4 DMA receive request
#define DMA_REQ_LPSPI5_TX    (DMAMUX_CHCFG_DMAMUX1 | 54) ///< LPSPI5 DMA transmit request
#define DMA_REQ_LPSPI5_RX    (DMAMUX_CHCFG_DMAMUX1 | 55) ///< LPSPI5 DMA receive request
#define DMA_REQ_SAI1_RX      (DMAMUX_CHCFG_DMAMUX1 | 56) ///< SAI1 DMA RX request
#define DMA_REQ_SAI1_TX      (DMAMUX_CHCFG_DMAMUX1 | 57) ///< SAI1 DMA TX request
#define DMA_REQ_RESERVED9    (DMAMUX_CHCFG_DMAMUX1 | 58) ///< RESERVED
#define DMA_REQ_RESERVED10   (DMAMUX_CHCFG_DMAMUX1 | 59) ///< RESERVED
#define DMA_REQ_LPCMP1       (DMAMUX_CHCFG_DMAMUX1 | 60) ///< LPCMP1 DMA request
#define DMA_REQ_LPCMP2       (DMAMUX_CHCFG_DMAMUX1 | 61) ///< LPCMP2 DMA request
#define DMA_REQ_ENABLED2     (DMAMUX_CHCFG_DMAMUX1 | 62) ///< Always enabled
#define DMA_REQ_ENABLED3     (DMAMUX_CHCFG_DMAMUX1 | 63) ///< Always enabled

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_DMAMUX_H */
