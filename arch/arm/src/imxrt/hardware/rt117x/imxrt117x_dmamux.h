/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_dmamux.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_DMAMUX_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Peripheral DMA request channels */

#define IMXRT_DMACHAN_RESERVED0        0  /* Reserved */
#define IMXRT_DMACHAN_FLEXIO1_23       1  /* FlexIO1 DMA 2/3, Async DMA 2/3 */
#define IMXRT_DMACHAN_FLEXIO1_45       2  /* FlexIO1 DMA 4/5, Async DMA 4/5 */
#define IMXRT_DMACHAN_FLEXIO1_67       3  /* FlexIO1 DMA 6/7, Async DMA 6/7 */
#define IMXRT_DMACHAN_FLEXIO2_01       4  /* FlexIO2 DMA 0/1, Async DMA 0/1 */
#define IMXRT_DMACHAN_FLEXIO2_23       5  /* FlexIO2 DMA 2/3, Async DMA 2/3 */
#define IMXRT_DMACHAN_FLEXIO2_45       6  /* FlexIO2 DMA 4/5, Async DMA 4/5 */
#define IMXRT_DMACHAN_FLEXIO2_67       7  /* FlexIO2 DMA 6/7, Async DMA 6/7 */
#define IMXRT_DMACHAN_LPUART1_TX       8  /* LPUART1 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART1_RX       9  /* LPUART1 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART2_TX      10  /* LPUART2 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART2_RX      11  /* LPUART2 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART3_TX      12  /* LPUART3 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART3_RX      13  /* LPUART3 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART4_TX      14  /* LPUART4 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART4_RX      15  /* LPUART4 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART5_TX      16  /* LPUART5 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART5_RX      17  /* LPUART5 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART6_TX      18  /* LPUART6 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART6_RX      19  /* LPUART6 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART7_TX      20  /* LPUART7 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART7_RX      21  /* LPUART7 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART8_TX      22  /* LPUART8 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART8_RX      23  /* LPUART8 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART9_TX      24  /* LPUART9 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART9_RX      25  /* LPUART9 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART10_TX     26  /* LPUART10 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART10_RX     27  /* LPUART10 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART11_TX     28  /* LPUART11 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART11_RX     29  /* LPUART11 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART12_TX     30  /* LPUART12 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPUART12_RX     31  /* LPUART12 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_CSI             32  /* CSI Write DMA */
#define IMXRT_DMACHAN_PXP             33  /* PXP DMA Event */
#define IMXRT_DMACHAN_ELCDIF          34  /* eLCDIF DMA Event */
#define IMXRT_DMACHAN_LCDIFV2         35  /* LCDIFv2 DMA Event */
#define IMXRT_DMACHAN_LPSPI1_RX       36  /* LPSPI1 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI1_TX       37  /* LPSPI1 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI2_RX       38  /* LPSPI2 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI2_TX       39  /* LPSPI2 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI3_RX       40  /* LPSPI3 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI3_TX       41  /* LPSPI3 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI4_RX       42  /* LPSPI4 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI4_TX       43  /* LPSPI4 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI5_RX       44  /* LPSPI5 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI5_TX       45  /* LPSPI5 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI6_RX       46  /* LPSPI6 RX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPSPI6_TX       47  /* LPSPI6 TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPI2C1          48  /* LPI2C1 Master/Slave RX/TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPI2C2          49  /* LPI2C2 Master/Slave RX/TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPI2C3          50  /* LPI2C3 Master/Slave RX/TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPI2C4          51  /* LPI2C4 Master/Slave RX/TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPI2C5          52  /* LPI2C5 Master/Slave RX/TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_LPI2C6          53  /* LPI2C6 Master/Slave RX/TX FIFO DMA / Async DMA */
#define IMXRT_DMACHAN_SAI1_RX         54  /* SAI1 RX FIFO DMA */
#define IMXRT_DMACHAN_SAI1_TX         55  /* SAI1 TX FIFO DMA */
#define IMXRT_DMACHAN_SAI2_RX         56  /* SAI2 RX FIFO DMA */
#define IMXRT_DMACHAN_SAI2_TX         57  /* SAI2 TX FIFO DMA */
#define IMXRT_DMACHAN_SAI3_RX         58  /* SAI3 RX FIFO DMA */
#define IMXRT_DMACHAN_SAI3_TX         59  /* SAI3 TX FIFO DMA */
#define IMXRT_DMACHAN_SAI4_RX         60  /* SAI4 RX FIFO DMA */
#define IMXRT_DMACHAN_SAI4_TX         61  /* SAI4 TX FIFO DMA */
#define IMXRT_DMACHAN_SPDIF_RX        62  /* SPDIF RX DMA */
#define IMXRT_DMACHAN_SPDIF_TX        63  /* SPDIF TX DMA */
#define IMXRT_DMACHAN_ADC_ETC         64  /* ADC_ETC DMA */
#define IMXRT_DMACHAN_FLEXIO1_01      65  /* FlexIO1 DMA 0/1, Async DMA 0/1 */
#define IMXRT_DMACHAN_LPADC1          66  /* ADC1 DMA */
#define IMXRT_DMACHAN_LPADC2          67  /* ADC2 DMA */
#define IMXRT_DMACHAN_RESERVED68      68  /* Reserved */
#define IMXRT_DMACHAN_ACMP1           69  /* ACMP1 DMA */
#define IMXRT_DMACHAN_ACMP2           70  /* ACMP2 DMA */
#define IMXRT_DMACHAN_ACMP3           71  /* ACMP3 DMA */
#define IMXRT_DMACHAN_ACMP4           72  /* ACMP4 DMA */
#define IMXRT_DMACHAN_RESERVED73      73  /* Reserved */
#define IMXRT_DMACHAN_RESERVED74      74  /* Reserved */
#define IMXRT_DMACHAN_RESERVED75      75  /* Reserved */
#define IMXRT_DMACHAN_RESERVED76      76  /* Reserved */
#define IMXRT_DMACHAN_FLEXSPI1_RX     77  /* FlexSPI1 RX FIFO DMA */
#define IMXRT_DMACHAN_FLEXSPI1_TX     78  /* FlexSPI1 TX FIFO DMA */
#define IMXRT_DMACHAN_FLEXSPI2_RX     79  /* FlexSPI2 RX FIFO DMA */
#define IMXRT_DMACHAN_FLEXSPI2_TX     80  /* FlexSPI2 TX FIFO DMA */
#define IMXRT_DMACHAN_XBAR1_0         81  /* XBAR1 DMA 0 */
#define IMXRT_DMACHAN_XBAR1_1         82  /* XBAR1 DMA 1 */
#define IMXRT_DMACHAN_XBAR1_2         83  /* XBAR1 DMA 2 */
#define IMXRT_DMACHAN_XBAR1_3         84  /* XBAR1 DMA 3 */
#define IMXRT_DMACHAN_FLEXPWM1_RX0    85  /* FlexPWM1 RX sub-module0 capture */
#define IMXRT_DMACHAN_FLEXPWM1_RX1    86  /* FlexPWM1 RX sub-module1 capture */
#define IMXRT_DMACHAN_FLEXPWM1_RX2    87  /* FlexPWM1 RX sub-module2 capture */
#define IMXRT_DMACHAN_FLEXPWM1_RX3    88  /* FlexPWM1 RX sub-module3 capture */
#define IMXRT_DMACHAN_FLEXPWM1_TX0    89  /* FlexPWM1 TX sub-module0 value */
#define IMXRT_DMACHAN_FLEXPWM1_TX1    90  /* FlexPWM1 TX sub-module1 value */
#define IMXRT_DMACHAN_FLEXPWM1_TX2    91  /* FlexPWM1 TX sub-module2 value */
#define IMXRT_DMACHAN_FLEXPWM1_TX3    92  /* FlexPWM1 TX sub-module3 value */
#define IMXRT_DMACHAN_FLEXPWM2_RX0    93  /* FlexPWM2 RX sub-module0 capture */
#define IMXRT_DMACHAN_FLEXPWM2_RX1    94  /* FlexPWM2 RX sub-module1 capture */
#define IMXRT_DMACHAN_FLEXPWM2_RX2    95  /* FlexPWM2 RX sub-module2 capture */
#define IMXRT_DMACHAN_FLEXPWM2_RX3    96  /* FlexPWM2 RX sub-module3 capture */
#define IMXRT_DMACHAN_FLEXPWM2_TX0    97  /* FlexPWM2 TX sub-module0 value */
#define IMXRT_DMACHAN_FLEXPWM2_TX1    98  /* FlexPWM2 TX sub-module1 value */
#define IMXRT_DMACHAN_FLEXPWM2_TX2    99  /* FlexPWM2 TX sub-module2 value */
#define IMXRT_DMACHAN_FLEXPWM2_TX3   100  /* FlexPWM2 TX sub-module3 value */
#define IMXRT_DMACHAN_FLEXPWM3_RX0   101  /* FlexPWM3 RX sub-module0 capture */
#define IMXRT_DMACHAN_FLEXPWM3_RX1   102  /* FlexPWM3 RX sub-module1 capture */
#define IMXRT_DMACHAN_FLEXPWM3_RX2   103  /* FlexPWM3 RX sub-module2 capture */
#define IMXRT_DMACHAN_FLEXPWM3_RX3   104  /* FlexPWM3 RX sub-module3 capture */
#define IMXRT_DMACHAN_FLEXPWM3_TX0   105  /* FlexPWM3 TX sub-module0 value */
#define IMXRT_DMACHAN_FLEXPWM3_TX1   106  /* FlexPWM3 TX sub-module1 value */
#define IMXRT_DMACHAN_FLEXPWM3_TX2   107  /* FlexPWM3 TX sub-module2 value */
#define IMXRT_DMACHAN_FLEXPWM3_TX3   108  /* FlexPWM3 TX sub-module3 value */
#define IMXRT_DMACHAN_FLEXPWM4_RX0   109  /* FlexPWM4 RX sub-module0 capture */
#define IMXRT_DMACHAN_FLEXPWM4_RX1   110  /* FlexPWM4 RX sub-module1 capture */
#define IMXRT_DMACHAN_FLEXPWM4_RX2   111  /* FlexPWM4 RX sub-module2 capture */
#define IMXRT_DMACHAN_FLEXPWM4_RX3   112  /* FlexPWM4 RX sub-module3 capture */
#define IMXRT_DMACHAN_FLEXPWM4_TX0   113  /* FlexPWM4 TX sub-module0 value */
#define IMXRT_DMACHAN_FLEXPWM4_TX1   114  /* FlexPWM4 TX sub-module1 value */
#define IMXRT_DMACHAN_FLEXPWM4_TX2   115  /* FlexPWM4 TX sub-module2 value */
#define IMXRT_DMACHAN_FLEXPWM4_TX3   116  /* FlexPWM4 TX sub-module3 value */
#define IMXRT_DMACHAN_RESERVED117    117  /* Reserved */
#define IMXRT_DMACHAN_RESERVED118    118  /* Reserved */
#define IMXRT_DMACHAN_RESERVED119    119  /* Reserved */
#define IMXRT_DMACHAN_RESERVED120    120  /* Reserved */
#define IMXRT_DMACHAN_RESERVED121    121  /* Reserved */
#define IMXRT_DMACHAN_RESERVED122    122  /* Reserved */
#define IMXRT_DMACHAN_RESERVED123    123  /* Reserved */
#define IMXRT_DMACHAN_RESERVED124    124  /* Reserved */
#define IMXRT_DMACHAN_RESERVED125    125  /* Reserved */
#define IMXRT_DMACHAN_RESERVED126    126  /* Reserved */
#define IMXRT_DMACHAN_RESERVED127    127  /* Reserved */
#define IMXRT_DMACHAN_RESERVED128    128  /* Reserved */
#define IMXRT_DMACHAN_RESERVED129    129  /* Reserved */
#define IMXRT_DMACHAN_RESERVED130    130  /* Reserved */
#define IMXRT_DMACHAN_RESERVED131    131  /* Reserved */
#define IMXRT_DMACHAN_RESERVED132    132  /* Reserved */
#define IMXRT_DMACHAN_QTIMER1_RX0    133  /* QTimer1 RX capture timer 0 */
#define IMXRT_DMACHAN_QTIMER1_RX1    134  /* QTimer1 RX capture timer 1 */
#define IMXRT_DMACHAN_QTIMER1_RX2    135  /* QTimer1 RX capture timer 2 */
#define IMXRT_DMACHAN_QTIMER1_RX3    136  /* QTimer1 RX capture timer 3 */
#define IMXRT_DMACHAN_QTIMER1_TX0    137  /* QTimer1 TX cmpld1 timer 0 / cmld2 timer 1 */
#define IMXRT_DMACHAN_QTIMER1_TX1    138  /* QTimer1 TX cmpld1 timer 1 / cmld2 timer 0 */
#define IMXRT_DMACHAN_QTIMER1_TX2    139  /* QTimer1 TX cmpld1 timer 2 / cmld2 timer 3 */
#define IMXRT_DMACHAN_QTIMER1_TX3    140  /* QTimer1 TX cmpld1 timer 3 / cmld2 timer 2 */
#define IMXRT_DMACHAN_QTIMER2_RX0    141  /* QTimer2 RX capture timer 0 */
#define IMXRT_DMACHAN_QTIMER2_RX1    142  /* QTimer2 RX capture timer 1 */
#define IMXRT_DMACHAN_QTIMER2_RX2    143  /* QTimer2 RX capture timer 2 */
#define IMXRT_DMACHAN_QTIMER2_RX3    144  /* QTimer2 RX capture timer 3 */
#define IMXRT_DMACHAN_QTIMER2_TX0    145  /* QTimer2 TX cmpld1 timer 0 / cmld2 timer 1 */
#define IMXRT_DMACHAN_QTIMER2_TX1    146  /* QTimer2 TX cmpld1 timer 1 / cmld2 timer 0 */
#define IMXRT_DMACHAN_QTIMER2_TX2    147  /* QTimer2 TX cmpld1 timer 2 / cmld2 timer 3 */
#define IMXRT_DMACHAN_QTIMER2_TX3    148  /* QTimer2 TX cmpld1 timer 3 / cmld2 timer 2 */
#define IMXRT_DMACHAN_QTIMER3_RX0    149  /* QTimer3 RX capture timer 0 */
#define IMXRT_DMACHAN_QTIMER3_RX1    150  /* QTimer3 RX capture timer 1 */
#define IMXRT_DMACHAN_QTIMER3_RX2    151  /* QTimer3 RX capture timer 2 */
#define IMXRT_DMACHAN_QTIMER3_RX3    152  /* QTimer3 RX capture timer 3 */
#define IMXRT_DMACHAN_QTIMER3_TX0    153  /* QTimer3 TX cmpld1 timer 0 / cmld2 timer 1 */
#define IMXRT_DMACHAN_QTIMER3_TX1    154  /* QTimer3 TX cmpld1 timer 1 / cmld2 timer 0 */
#define IMXRT_DMACHAN_QTIMER3_TX2    155  /* QTimer3 TX cmpld1 timer 2 / cmld2 timer 3 */
#define IMXRT_DMACHAN_QTIMER3_TX3    156  /* QTimer3 TX cmpld1 timer 3 / cmld2 timer 2 */
#define IMXRT_DMACHAN_QTIMER4_RX0    157  /* QTimer4 RX capture timer 0 */
#define IMXRT_DMACHAN_QTIMER4_RX1    158  /* QTimer4 RX capture timer 1 */
#define IMXRT_DMACHAN_QTIMER4_RX2    159  /* QTimer4 RX capture timer 2 */
#define IMXRT_DMACHAN_QTIMER4_RX3    160  /* QTimer4 RX capture timer 3 */
#define IMXRT_DMACHAN_QTIMER4_TX0    161  /* QTimer4 TX cmpld1 timer 0 / cmld2 timer 1 */
#define IMXRT_DMACHAN_QTIMER4_TX1    162  /* QTimer4 TX cmpld1 timer 1 / cmld2 timer 0 */
#define IMXRT_DMACHAN_QTIMER4_TX2    163  /* QTimer4 TX cmpld1 timer 2 / cmld2 timer 3 */
#define IMXRT_DMACHAN_QTIMER4_TX3    164  /* QTimer4 TX cmpld1 timer 3 / cmld2 timer 2 */
#define IMXRT_DMACHAN_RESERVED165    165  /* Reserved */
#define IMXRT_DMACHAN_RESERVED166    166  /* Reserved */
#define IMXRT_DMACHAN_RESERVED167    167  /* Reserved */
#define IMXRT_DMACHAN_RESERVED168    168  /* Reserved */
#define IMXRT_DMACHAN_RESERVED169    169  /* Reserved */
#define IMXRT_DMACHAN_RESERVED170    170  /* Reserved */
#define IMXRT_DMACHAN_RESERVED171    171  /* Reserved */
#define IMXRT_DMACHAN_RESERVED172    172  /* Reserved */
#define IMXRT_DMACHAN_RESERVED173    173  /* Reserved */
#define IMXRT_DMACHAN_RESERVED174    174  /* Reserved */
#define IMXRT_DMACHAN_RESERVED175    175  /* Reserved */
#define IMXRT_DMACHAN_RESERVED176    176  /* Reserved */
#define IMXRT_DMACHAN_RESERVED177    177  /* Reserved */
#define IMXRT_DMACHAN_RESERVED178    178  /* Reserved */
#define IMXRT_DMACHAN_RESERVED179    179  /* Reserved */
#define IMXRT_DMACHAN_RESERVED180    180  /* Reserved */
#define IMXRT_DMACHAN_PDM            181  /* PDM DMA / Async DMA */
#define IMXRT_DMACHAN_ENET_0         182  /* ENET Timer DMA 0 */
#define IMXRT_DMACHAN_ENET_1         183  /* ENET Timer DMA 1 */
#define IMXRT_DMACHAN_ENET2_0        184  /* ENET_1G Timer DMA 0 */
#define IMXRT_DMACHAN_ENET2_1        185  /* ENET_1G Timer DMA 1 */
#define IMXRT_DMACHAN_CAN1           186  /* FLEXCAN1 DMA */
#define IMXRT_DMACHAN_CAN2           187  /* FLEXCAN2 DMA */
#define IMXRT_DMACHAN_CAN3           188  /* FLEXCAN3 DMA */
#define IMXRT_DMACHAN_DAC            189  /* DAC DMA */
#define IMXRT_DMACHAN_RESERVED190    190  /* Reserved */
#define IMXRT_DMACHAN_ASRC_INA       191  /* ASRC pair A input */
#define IMXRT_DMACHAN_ASRC_INB       192  /* ASRC pair B input */
#define IMXRT_DMACHAN_ASRC_INC       193  /* ASRC pair C input */
#define IMXRT_DMACHAN_ASRC_OUTA      194  /* ASRC pair A output */
#define IMXRT_DMACHAN_ASRC_OUTB      195  /* ASRC pair B output */
#define IMXRT_DMACHAN_ASRC_OUTC      196  /* ASRC pair C output */
#define IMXRT_DMACHAN_EMVSIM1_TX     197  /* EMVSIM1 TX DMA */
#define IMXRT_DMACHAN_EMVSIM1_RX     198  /* EMVSIM1 RX DMA */
#define IMXRT_DMACHAN_EMVSIM2_TX     199  /* EMVSIM2 TX DMA */
#define IMXRT_DMACHAN_EMVSIM2_RX     200  /* EMVSIM2 RX DMA */
#define IMXRT_DMACHAN_ENET3_0        201  /* ENET_QOS PPS output DMA 0 */
#define IMXRT_DMACHAN_ENET3_1        202  /* ENET_QOS PPS output DMA 1 */
#define IMXRT_DMACHAN_RESERVED203    203  /* Reserved */
#define IMXRT_DMACHAN_RESERVED204    204  /* Reserved */
#define IMXRT_DMACHAN_RESERVED205    205  /* Reserved */
#define IMXRT_DMACHAN_RESERVED206    206  /* Reserved */
#define IMXRT_DMACHAN_RESERVED207    207  /* Reserved */

#define IMXRT_DMA_NCHANNELS          208  /* Includes reserved channels */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_DMAMUX_H */
