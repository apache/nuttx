/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k11x_dmamux.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K11X_DMAMUX_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K11X_DMAMUX_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Peripheral DMA request channels */

#define S32K1XX_DMACHAN_DISABLED                   0  /* Disabled DMA Request */
#define S32K1XX_DMACHAN_LPUART0_RX                 2  /* lpuart0  Receive DMA Request */
#define S32K1XX_DMACHAN_LPUART0_TX                 3  /* lpuart0  Transmit DMA Request */
#define S32K1XX_DMACHAN_LPUART1_RX                 4  /* lpuart1  Receive DMA Request */
#define S32K1XX_DMACHAN_LPUART1_TX                 5  /* lpuart1  Transmit DMA Request */
#define S32K1XX_DMACHAN_FLEXIO_SHIFTER0            10 /* flexio FlexIO Shifter0 DMA Request */
#define S32K1XX_DMACHAN_FLEXIO_SHIFTER1            11 /* flexio FlexIO Shifter1 DMA Request */
#define S32K1XX_DMACHAN_FLEXIO_SHIFTER2            12 /* flexio / SAI1  FlexIO Shifter2 DMA Request / SAI1 DMA RX Request */
#define S32K1XX_DMACHAN_FLEXIO_SHIFTER3            13 /* flexio / SAI1  FlexIO Shifter3 DMA Request / SAI1 DMA TX Request */
#define S32K1XX_DMACHAN_LPSPI0_RX                  14 /* lpspi0 DMA RX Request */
#define S32K1XX_DMACHAN_LPSPI0_TX                  15 /* lpspi0 DMA TX Request */
#define S32K1XX_DMACHAN_LPSPI1_RX                  16 /* lpspi1 DMA RX Request */
#define S32K1XX_DMACHAN_LPSPI1_TX                  17 /* lpspi1 DMA TX Request */
#define S32K1XX_DMACHAN_FTM1_CHANNEL_0             20 /* ftm1 Channel 0 DMA transfer request */
#define S32K1XX_DMACHAN_FTM1_CHANNEL_1             21 /* ftm1 Channel 1 DMA transfer request */
#define S32K1XX_DMACHAN_FTM1_CHANNEL_2             22 /* ftm1 Channel 2 DMA transfer request */
#define S32K1XX_DMACHAN_FTM1_CHANNEL_3             23 /* ftm1 Channel 3 DMA transfer request */
#define S32K1XX_DMACHAN_FTM1_CHANNEL_4             24 /* ftm1 Channel 4 DMA transfer request */
#define S32K1XX_DMACHAN_FTM1_CHANNEL_5             25 /* ftm1 Channel 5 DMA transfer request */
#define S32K1XX_DMACHAN_FTM1_CHANNEL_6             26 /* ftm1 Channel 6 DMA transfer request */
#define S32K1XX_DMACHAN_FTM1_CHANNEL_7             27 /* ftm1 Channel 7 DMA transfer request */
#define S32K1XX_DMACHAN_FTM0_OR_CH0_CH7            36 /* ftm0 'OR' of FTM0 channel 0 - 7 */
#define S32K1XX_DMACHAN_ADC0                       42 /* ftm6 'OR' of FTM6 channel 0 - 7 */
#define S32K1XX_DMACHAN_LPI2C0_RX                  44 /* lpi2c0 DMA RX Request */
#define S32K1XX_DMACHAN_LPI2C0_TX                  45 /* lpi2c0 DMA TX Request */
#define S32K1XX_DMACHAN_PDB0                       46 /* pdb0 DMA request */
#define S32K1XX_DMACHAN_CMP0                       48 /* cmp0 DMA request */
#define S32K1XX_DMACHAN_PORTA                      49 /* PORT PORTA DMA request */
#define S32K1XX_DMACHAN_PORTB                      50 /* PORT PORTB DMA request */
#define S32K1XX_DMACHAN_PORTC                      51 /* PORT PORTC DMA request */
#define S32K1XX_DMACHAN_PORTD                      52 /* PORT PORTD DMA request */
#define S32K1XX_DMACHAN_PORTE                      53 /* PORT PORTE DMA request */
#define S32K1XX_DMACHAN_FLEXCAN0                   54 /* flexcan0 DMA request */
#define S32K1XX_DMACHAN_LPTMR0                     59 /* lptmr0 LPTIMER DMA request */
#define S32K1XX_DMACHAN_DMAMUX_ALWAYS_ENABLED0     62 /* Always On DMA request */
#define S32K1XX_DMACHAN_DMAMUX_ALWAYS_ENABLED1     63 /* Always On DMA request */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K11X_DMAMUX_H */
