/****************************************************************************
 * arch/arm/include/rm57/rm57l843_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

/* VIM channel -> peripheral request assignment is a fixed hardware
 * property, extracted directly from TI's HALCoGen project file rather
 * than assumed from TMS570 parity. The project's VIM_CHANNELx_MAPPING
 * values confirm HALCoGen configured the default identity mapping
 * (channel N carries request N) - if a future board re-maps channels
 * via vimChannelMap(), this table describes the *request* identity, and
 * the active *channel* routing must be re-derived from that call.
 *
 * Channel 127 has no dedicated request (always phantomInterrupt) and
 * must not be used, mirroring the sibling TMS570_IRQ_NCHANNELS pattern.
 */

#ifndef __ARCH_ARM_INCLUDE_RM57_RM57L843_IRQ_H
#define __ARCH_ARM_INCLUDE_RM57_RM57L843_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define RM57_IRQ_NCHANNELS   127 /* The "phantom" vector is followed by 127
                                   * real channels; channel 127 is reserved
                                   * (always phantomInterrupt) */

/* Default channel (=request) assignments, from RM57L_Contents.dil */

#define RM57_REQ_ESMHIGH             0  /* ESM High level interrupt (FIQ) */
#define RM57_REQ_RTICOMPARE0         2  /* RTI compare interrupt 0 */
#define RM57_REQ_RTICOMPARE1         3  /* RTI compare interrupt 1 */
#define RM57_REQ_RTICOMPARE2         4  /* RTI compare interrupt 2 */
#define RM57_REQ_RTICOMPARE3         5  /* RTI compare interrupt 3 */
#define RM57_REQ_RTIOVERFLOW0        6  /* RTI overflow interrupt 0 */
#define RM57_REQ_RTIOVERFLOW1        7  /* RTI overflow interrupt 1 */
#define RM57_REQ_RTITIMEBASE         8  /* RTI timebase interrupt */
#define RM57_REQ_GIOHIGH             9  /* GIO high level interrupt */
#define RM57_REQ_HET1HIGH            10 /* HET1 high level interrupt */
#define RM57_REQ_MIBSPI1HIGH         12 /* MibSPI1 high level interrupt */
#define RM57_REQ_LIN1HIGH            13 /* LIN1/SCI1 high level interrupt */
#define RM57_REQ_ADC1GROUP0          14 /* ADC1 group 0 interrupt */
#define RM57_REQ_ADC1GROUP1          15 /* ADC1 group 1 interrupt */
#define RM57_REQ_CAN1HIGH            16 /* DCAN1 high level interrupt */
#define RM57_REQ_SPI2HIGH            17 /* SPI2 high level interrupt */
#define RM57_REQ_CRC                 19 /* CRC interrupt */
#define RM57_REQ_ESMLOW              20 /* ESM Low level interrupt */
#define RM57_REQ_GIOLOW              23 /* GIO low level interrupt */
#define RM57_REQ_HET1LOW             24 /* HET1 low level interrupt */
#define RM57_REQ_MIBSPI1LOW          26 /* MibSPI1 low level interrupt */
#define RM57_REQ_LIN1LOW             27 /* LIN1/SCI1 low level interrupt */
#define RM57_REQ_ADC1GROUP2          28 /* ADC1 group 2 interrupt */
#define RM57_REQ_CAN1LOW             29 /* DCAN1 low level interrupt */
#define RM57_REQ_MIBSPI2LOW          30 /* MibSPI2 low level interrupt */
#define RM57_REQ_ADC1MAGCOMP         31 /* MibADC1 magnitude compare */
#define RM57_REQ_DMAFTCA             33 /* DMA FTCA interrupt */
#define RM57_REQ_DMALFSA             34 /* DMA LFSA interrupt */
#define RM57_REQ_CAN2HIGH            35 /* DCAN2 high level interrupt */
#define RM57_REQ_MIBSPI3HIGH         37 /* MibSPI3 high level interrupt */
#define RM57_REQ_MIBSPI3LOW          38 /* MibSPI3 low level interrupt */
#define RM57_REQ_DMAHBCA             39 /* DMA HBCA interrupt */
#define RM57_REQ_DMABTCA             40 /* DMA BTCA interrupt */
#define RM57_REQ_CAN2LOW             42 /* DCAN2 low level interrupt */
#define RM57_REQ_CAN1IF3             44 /* DCAN1 IF3 interrupt */
#define RM57_REQ_CAN3HIGH            45 /* DCAN3 high level interrupt */
#define RM57_REQ_CAN2IF3             46 /* DCAN2 IF3 interrupt */
#define RM57_REQ_MIBSPI4HIGH         49 /* MibSPI4 high level interrupt */
#define RM57_REQ_ADC2GROUP0          50 /* ADC2 group 0 interrupt */
#define RM57_REQ_ADC2GROUP1          51 /* ADC2 group 1 interrupt */
#define RM57_REQ_MIBSPI5HIGH         53 /* MibSPI5 high level interrupt */
#define RM57_REQ_MIBSPI4LOW          54 /* MibSPI4 low level interrupt */
#define RM57_REQ_CAN3LOW             55 /* DCAN3 low level interrupt */
#define RM57_REQ_MIBSPI5LOW          56 /* MibSPI5 low level interrupt */
#define RM57_REQ_ADC2GROUP2          57 /* ADC2 group 2 interrupt */
#define RM57_REQ_ADC2MAGCOMP         59 /* MibADC2 magnitude compare */
#define RM57_REQ_CAN3IF3             60 /* DCAN3 IF3 interrupt */
#define RM57_REQ_HET2HIGH            63 /* HET2 high level interrupt */
#define RM57_REQ_SCI3HIGH            64 /* SCI3 high level interrupt */
#define RM57_REQ_I2C                 66 /* I2C interrupt */
#define RM57_REQ_HET2LOW             73 /* HET2 low level interrupt */

/* Channel 74 pairs with RM57_REQ_SCI3HIGH (64) above - it is SCI3's low
 * level interrupt (SCI3 has no LIN capability, hence "LIN-less" in the
 * original HALCoGen project comment this was transcribed from; that
 * phrasing does not fit SCI2/LIN2, which is a full dual-role module).
 */
#define RM57_REQ_SCI3LOW             74 /* SCI3 low level interrupt */

/* Channels 76-79 are the four EMAC Control Module interrupt pulses
 * (SPNS215 Table 6-39): C0_MISC_PULSE, C0_TX_PULSE, C0_THRESH_PULSE and
 * C0_RX_PULSE, in that channel order.  C0_MISC_PULSE carries STATPEND,
 * HOSTPEND and both MDIO interrupts (LINKINT0, USERINT0); see
 * hardware/rm57_ethernet.h for the EMAC_MACINVECTOR/MACEOIVECTOR decode.
 */

#define RM57_REQ_EMACMISC             76 /* EMAC C0_MISC_PULSE (stats/host/MDIO) */
#define RM57_REQ_EMACTX               77 /* EMAC C0_TX_PULSE (Tx completion) */
#define RM57_REQ_EMACTHRESH           78 /* EMAC C0_THRESH_PULSE (Rx flow threshold) */
#define RM57_REQ_EMACRX               79 /* EMAC C0_RX_PULSE (Rx completion) */
#define RM57_REQ_DCC1DONE            82  /* DCC1 done interrupt */
#define RM57_REQ_DCC2DONE            83  /* DCC2 done interrupt */
#define RM57_REQ_ETPWM1              90  /* ETPWM1 interrupt */
#define RM57_REQ_ETPWM1TZ            91  /* ETPWM1 trip-zone interrupt */
#define RM57_REQ_ETPWM2              92  /* ETPWM2 interrupt */
#define RM57_REQ_ETPWM2TZ            93  /* ETPWM2 trip-zone interrupt */
#define RM57_REQ_ETPWM3              94  /* ETPWM3 interrupt */
#define RM57_REQ_ETPWM3TZ            95  /* ETPWM3 trip-zone interrupt */
#define RM57_REQ_ETPWM4              96  /* ETPWM4 interrupt */
#define RM57_REQ_ETPWM4TZ            97  /* ETPWM4 trip-zone interrupt */
#define RM57_REQ_ETPWM5              98  /* ETPWM5 interrupt */
#define RM57_REQ_ETPWM5TZ            99  /* ETPWM5 trip-zone interrupt */
#define RM57_REQ_ETPWM6             100  /* ETPWM6 interrupt */
#define RM57_REQ_ETPWM6TZ           101  /* ETPWM6 trip-zone interrupt */
#define RM57_REQ_ETPWM7             102  /* ETPWM7 interrupt */
#define RM57_REQ_ETPWM7TZ           103  /* ETPWM7 trip-zone interrupt */
#define RM57_REQ_ECAP1              104  /* ECAP1 interrupt */
#define RM57_REQ_ECAP2              105  /* ECAP2 interrupt */
#define RM57_REQ_ECAP3              106  /* ECAP3 interrupt */
#define RM57_REQ_ECAP4              107  /* ECAP4 interrupt */
#define RM57_REQ_ECAP5              108  /* ECAP5 interrupt */
#define RM57_REQ_ECAP6              109  /* ECAP6 interrupt */
#define RM57_REQ_EQEP1              110  /* EQEP1 interrupt */
#define RM57_REQ_EQEP2              111  /* EQEP2 interrupt */
#define RM57_REQ_CAN4HIGH           113  /* DCAN4 high level interrupt */
#define RM57_REQ_I2C2               114  /* I2C2 interrupt */
#define RM57_REQ_LIN2HIGH           115  /* LIN2 high level interrupt */
#define RM57_REQ_SCI4HIGH           116  /* SCI4 high level interrupt */
#define RM57_REQ_CAN4LOW            117  /* DCAN4 low level interrupt */
#define RM57_REQ_LIN2LOW            118  /* LIN2 low level interrupt */
#define RM57_REQ_SCI4LOW            119  /* SCI4 low level interrupt */
#define RM57_REQ_CAN4IF3            120  /* DCAN4 IF3 interrupt */
#define RM57_REQ_CRC2               121  /* CRC2 interrupt */
#define RM57_REQ_EPCFULL            124  /* EPC FIFO full interrupt */

/* Second-level GIO pin interrupt demux, enabled by CONFIG_RM57_GIO_IRQ.
 * RM57L843 has only 2 GIO ports (A, B), unlike TMS570's up to 4/8, so
 * only GIOA0-7/GIOB0-7 are allocated here.
 *
 * These numbers start at RM57_IRQ_NCHANNELS (127), which is free because
 * VIM channels 0-126 occupy IRQs 0-126 and channel 127 is reserved.  That
 * keeps the whole block inside NR_IRQS, which irq.h computes as
 * RM57_IRQ_NCHANNELS + RM57_NGIO_IRQS.  The sibling TMS570 header bases
 * this block at NCHANNELS+1 instead, which pushes its last GIO IRQ one
 * past NR_IRQS where irq_attach()/irq_dispatch() reject it.
 */

#ifdef CONFIG_RM57_GIO_IRQ
#  define RM57_IRQ_GIOA0            (RM57_IRQ_NCHANNELS+0)
#  define RM57_IRQ_GIOA1            (RM57_IRQ_NCHANNELS+1)
#  define RM57_IRQ_GIOA2            (RM57_IRQ_NCHANNELS+2)
#  define RM57_IRQ_GIOA3            (RM57_IRQ_NCHANNELS+3)
#  define RM57_IRQ_GIOA4            (RM57_IRQ_NCHANNELS+4)
#  define RM57_IRQ_GIOA5            (RM57_IRQ_NCHANNELS+5)
#  define RM57_IRQ_GIOA6            (RM57_IRQ_NCHANNELS+6)
#  define RM57_IRQ_GIOA7            (RM57_IRQ_NCHANNELS+7)

#  define RM57_IRQ_GIOB0            (RM57_IRQ_NCHANNELS+8)
#  define RM57_IRQ_GIOB1            (RM57_IRQ_NCHANNELS+9)
#  define RM57_IRQ_GIOB2            (RM57_IRQ_NCHANNELS+10)
#  define RM57_IRQ_GIOB3            (RM57_IRQ_NCHANNELS+11)
#  define RM57_IRQ_GIOB4            (RM57_IRQ_NCHANNELS+12)
#  define RM57_IRQ_GIOB5            (RM57_IRQ_NCHANNELS+13)
#  define RM57_IRQ_GIOB6            (RM57_IRQ_NCHANNELS+14)
#  define RM57_IRQ_GIOB7            (RM57_IRQ_NCHANNELS+15)

#  define RM57_NGIO_IRQS            16
#else
#  define RM57_NGIO_IRQS            0
#endif

#endif /* __ARCH_ARM_INCLUDE_RM57_RM57L843_IRQ_H */
