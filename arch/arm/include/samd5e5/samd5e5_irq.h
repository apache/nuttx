/****************************************************************************
 * arch/arm/include/samd5e5/samd5e5_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_SAMD5E5_SAMD5E5_IRQ_H
#define __ARCH_ARM_INCLUDE_SAMD5E5_SAMD5E5_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* External interrupts (vectors >= 16) */

#define SAM_IRQ_PM             (SAM_IRQ_EXTINT + 0)    /* 0   Power Manager: SLEEPRDY */
#define SAM_IRQ_MCLK           (SAM_IRQ_EXTINT + 1)    /* 1   Main clock: CKRDY */
#define SAM_IRQ_XOSC0          (SAM_IRQ_EXTINT + 2)    /* 2   XOSC0: Fail/Ready */
#define SAM_IRQ_XOSC1          (SAM_IRQ_EXTINT + 3)    /* 3   XOSC1: Fail/Ready */
#define SAM_IRQ_DFLL           (SAM_IRQ_EXTINT + 4)    /* 4   OSCCTRLD: FLLLOCKC, DFLLLOCKF,
                                                        *     DFLLOOB, DFLLRCS, DFLLRDY */
#define SAM_IRQ_DPLL0          (SAM_IRQ_EXTINT + 5)    /* 5   DPLL0: DPLLLCKF, DPLLLCKR,
                                                        *     DPLLLDRTO, DPLLLTO */
#define SAM_IRQ_DPLL1          (SAM_IRQ_EXTINT + 6)    /* 6   DPLL1: DPLLLCKF, DPLLLCKR,
                                                        *     DPLLLDRTO, DPLLLTO */
#define SAM_IRQ_OSC32K         (SAM_IRQ_EXTINT + 7)    /* 7   OSC32KCTRL: OSC32KRDY,
                                                        *     XOSC32KFAIL, XOSC32KRDY */
#define SAM_IRQ_SUPCRDY        (SAM_IRQ_EXTINT + 8)    /* 8   Supply Controller: BOD12RDY,
                                                        *     BOD33RDY, B12SRDY, B33SRDY,
                                                        *     VCORERDY, VREGRDY */
#define SAM_IRQ_SUPCDET        (SAM_IRQ_EXTINT + 9)    /* 9   Supply Controller: BOD12DET,
                                                        *     BOD33DET */
#define SAM_IRQ_WDT            (SAM_IRQ_EXTINT + 10)   /* 10  WDT: EW */
#define SAM_IRQ_RTC            (SAM_IRQ_EXTINT + 11)   /* 11  RTC, CMPA0-3, PERA0-7, TAMPERA */
#define SAM_IRQ_EXTINT0        (SAM_IRQ_EXTINT + 12)   /* 12  EIC: EXTINT0 */
#define SAM_IRQ_EXTINT1        (SAM_IRQ_EXTINT + 13)   /* 13  EIC: EXTINT1 */
#define SAM_IRQ_EXTINT2        (SAM_IRQ_EXTINT + 14)   /* 14  EIC: EXTINT2 */
#define SAM_IRQ_EXTINT3        (SAM_IRQ_EXTINT + 15)   /* 15  EIC: EXTINT3 */
#define SAM_IRQ_EXTINT4        (SAM_IRQ_EXTINT + 16)   /* 16  EIC: EXTINT4 */
#define SAM_IRQ_EXTINT5        (SAM_IRQ_EXTINT + 17)   /* 17  EIC: EXTINT5 */
#define SAM_IRQ_EXTINT6        (SAM_IRQ_EXTINT + 18)   /* 18  EIC: EXTINT6 */
#define SAM_IRQ_EXTINT7        (SAM_IRQ_EXTINT + 19)   /* 19  EIC: EXTINT7 */
#define SAM_IRQ_EXTINT8        (SAM_IRQ_EXTINT + 20)   /* 20  EIC: EXTINT8 */
#define SAM_IRQ_EXTINT9        (SAM_IRQ_EXTINT + 21)   /* 21  EIC: EXTINT9 */
#define SAM_IRQ_EXTINT10       (SAM_IRQ_EXTINT + 22)   /* 22  EIC: EXTINT10 */
#define SAM_IRQ_EXTINT11       (SAM_IRQ_EXTINT + 23)   /* 23  EIC: EXTINT11 */
#define SAM_IRQ_EXTINT12       (SAM_IRQ_EXTINT + 24)   /* 24  EIC: EXTINT12 */
#define SAM_IRQ_EXTINT13       (SAM_IRQ_EXTINT + 25)   /* 25  EIC: EXTINT13 */
#define SAM_IRQ_EXTINT14       (SAM_IRQ_EXTINT + 26)   /* 26  EIC: EXTINT14 */
#define SAM_IRQ_EXTINT15       (SAM_IRQ_EXTINT + 27)   /* 27  EIC: EXTINT15 */
#define SAM_IRQ_FREQM          (SAM_IRQ_EXTINT + 28)   /* 28  FREQM: Done */
#define SAM_IRQ_NVMCTRL0       (SAM_IRQ_EXTINT + 29)   /* 29  NVMCTRL: INTFLAG[0-7] */
#define SAM_IRQ_NVMCTRL1       (SAM_IRQ_EXTINT + 30)   /* 30  NVMCTRL: INTFLAG[8-10] */
#define SAM_IRQ_DMACH0         (SAM_IRQ_EXTINT + 31)   /* 31  DMA Channel 0: SUSP, TCMPL, TERR */
#define SAM_IRQ_DMACH1         (SAM_IRQ_EXTINT + 32)   /* 32  DMA Channel 1: SUSP, TCMPL, TERR */
#define SAM_IRQ_DMACH2         (SAM_IRQ_EXTINT + 33)   /* 33  DMA Channel 2: SUSP, TCMPL, TERR */
#define SAM_IRQ_DMACH3         (SAM_IRQ_EXTINT + 34)   /* 34  DMA Channel 3: SUSP, TCMPL, TERR */
#define SAM_IRQ_DMACH4_31      (SAM_IRQ_EXTINT + 35)   /* 35  DMA Channels 4-31: SUSP, TCMPL, TERR */
#define SAM_IRQ_EVSYS0         (SAM_IRQ_EXTINT + 36)   /* 36  EVSYS Channel 0: EVD, OVR */
#define SAM_IRQ_EVSYS1         (SAM_IRQ_EXTINT + 37)   /* 37  EVSYS Channel 1: EVD, OVR */
#define SAM_IRQ_EVSYS2         (SAM_IRQ_EXTINT + 38)   /* 38  EVSYS Channel 2: EVD, OVR */
#define SAM_IRQ_EVSYS3         (SAM_IRQ_EXTINT + 39)   /* 39  EVSYS Channel 3: EVD, OVR */
#define SAM_IRQ_EVSYS4_11      (SAM_IRQ_EXTINT + 40)   /* 40  EVSYS Channels 4-11: EVD, OVR */
#define SAM_IRQ_PAC            (SAM_IRQ_EXTINT + 41)   /* 41  PAC: ERR */
#define SAM_IRQ_RAMECC         (SAM_IRQ_EXTINT + 45)   /* 45  RAM ECC: 0-1 */
#define SAM_IRQ_SERCOM0_0      (SAM_IRQ_EXTINT + 46)   /* 46  SERCOM0: INTFLAG[0] */
#define SAM_IRQ_SERCOM0_1      (SAM_IRQ_EXTINT + 47)   /* 47  SERCOM0: INTFLAG[1] */
#define SAM_IRQ_SERCOM0_2      (SAM_IRQ_EXTINT + 48)   /* 48  SERCOM0: INTFLAG[2] */
#define SAM_IRQ_SERCOM0_46     (SAM_IRQ_EXTINT + 49)   /* 49  SERCOM0: INTFLAG[3-6] */
#define SAM_IRQ_SERCOM1_0      (SAM_IRQ_EXTINT + 50)   /* 50  SERCOM1: INTFLAG[0] */
#define SAM_IRQ_SERCOM1_1      (SAM_IRQ_EXTINT + 51)   /* 51  SERCOM1: INTFLAG[1] */
#define SAM_IRQ_SERCOM1_2      (SAM_IRQ_EXTINT + 52)   /* 52  SERCOM1: INTFLAG[2] */
#define SAM_IRQ_SERCOM1_46     (SAM_IRQ_EXTINT + 53)   /* 53  SERCOM1: INTFLAG[3-6] */
#define SAM_IRQ_SERCOM2_0      (SAM_IRQ_EXTINT + 54)   /* 54  SERCOM2: INTFLAG[0] */
#define SAM_IRQ_SERCOM2_1      (SAM_IRQ_EXTINT + 55)   /* 55  SERCOM2: INTFLAG[1] */
#define SAM_IRQ_SERCOM2_2      (SAM_IRQ_EXTINT + 56)   /* 56  SERCOM2: INTFLAG[2] */
#define SAM_IRQ_SERCOM2_46     (SAM_IRQ_EXTINT + 57)   /* 57  SERCOM2: INTFLAG[3-6] */
#define SAM_IRQ_SERCOM3_0      (SAM_IRQ_EXTINT + 58)   /* 58  SERCOM3: INTFLAG[0] */
#define SAM_IRQ_SERCOM3_1      (SAM_IRQ_EXTINT + 59)   /* 59  SERCOM3: INTFLAG[1] */
#define SAM_IRQ_SERCOM3_2      (SAM_IRQ_EXTINT + 60)   /* 60  SERCOM3: INTFLAG[2] */
#define SAM_IRQ_SERCOM3_46     (SAM_IRQ_EXTINT + 61)   /* 61  SERCOM3: INTFLAG[3-6] */
#define SAM_IRQ_SERCOM4_0      (SAM_IRQ_EXTINT + 62)   /* 62  SERCOM4: INTFLAG[0] */
#define SAM_IRQ_SERCOM4_1      (SAM_IRQ_EXTINT + 63)   /* 63  SERCOM4: INTFLAG[1] */
#define SAM_IRQ_SERCOM4_2      (SAM_IRQ_EXTINT + 64)   /* 64  SERCOM4: INTFLAG[2] */
#define SAM_IRQ_SERCOM4_46     (SAM_IRQ_EXTINT + 65)   /* 65  SERCOM4: INTFLAG[3-6] */
#define SAM_IRQ_SERCOM5_0      (SAM_IRQ_EXTINT + 66)   /* 66  SERCOM5: INTFLAG[0] */
#define SAM_IRQ_SERCOM5_1      (SAM_IRQ_EXTINT + 67)   /* 67  SERCOM5: INTFLAG[1] */
#define SAM_IRQ_SERCOM5_2      (SAM_IRQ_EXTINT + 68)   /* 68  SERCOM5: INTFLAG[2] */
#define SAM_IRQ_SERCOM5_46     (SAM_IRQ_EXTINT + 69)   /* 69  SERCOM5: INTFLAG[3-6] */
#define SAM_IRQ_SERCOM6_0      (SAM_IRQ_EXTINT + 70)   /* 70  SERCOM6: INTFLAG[0] */
#define SAM_IRQ_SERCOM6_1      (SAM_IRQ_EXTINT + 71)   /* 71  SERCOM6: INTFLAG[1] */
#define SAM_IRQ_SERCOM6_2      (SAM_IRQ_EXTINT + 72)   /* 72  SERCOM6: INTFLAG[2] */
#define SAM_IRQ_SERCOM6_46     (SAM_IRQ_EXTINT + 73)   /* 73  SERCOM6: INTFLAG[3-6] */
#define SAM_IRQ_SERCOM7_0      (SAM_IRQ_EXTINT + 74)   /* 74  SERCOM7: INTFLAG[0] */
#define SAM_IRQ_SERCOM7_1      (SAM_IRQ_EXTINT + 75)   /* 75  SERCOM7: INTFLAG[1] */
#define SAM_IRQ_SERCOM7_2      (SAM_IRQ_EXTINT + 76)   /* 76  SERCOM7: INTFLAG[2] */
#define SAM_IRQ_SERCOM7_46     (SAM_IRQ_EXTINT + 77)   /* 77  SERCOM7: INTFLAG[3-6] */
#define SAM_IRQ_CAN0           (SAM_IRQ_EXTINT + 78)   /* 78  CAN0: Line0, Line1 */
#define SAM_IRQ_CAN1           (SAM_IRQ_EXTINT + 79)   /* 79  CAN1: Line0, Line1 */
#define SAM_IRQ_USB            (SAM_IRQ_EXTINT + 80)   /* 80  USB: EORSM, DNRSM, EORST RST,
                                                        *     LPM DCONN, LPMSUSP DDISC, MSOF,
                                                        *     RAMACER, RXSTP TXSTP 0-7, STALL0
                                                        *     STALL 0-7, STALL1 0-7, SUSPEND,
                                                        *     TRFAIL0 TRFAIL 097, TRFAIL1 PERR
                                                        *      0..7, UPRSM, WAKEUP */
#define SAM_IRQ_USBSOF         (SAM_IRQ_EXTINT + 81)   /* 81  USB: SOF HSOF */
#define SAM_IRQ_USBTRCPT0      (SAM_IRQ_EXTINT + 82)   /* 82  USB: TRCPT0 0..7 */
#define SAM_IRQ_USBTRCPT1      (SAM_IRQ_EXTINT + 83)   /* 83  USB: TRCPT0 0..7 */
#define SAM_IRQ_GMAL           (SAM_IRQ_EXTINT + 84)   /* 84  GMAC:  GMAC, WOL */
#define SAM_IRQ_TCC0           (SAM_IRQ_EXTINT + 85)   /* 85  TCC0: CNT A, DFS A, ERR A, FAULTA
                                                        *     A, FAULTB A, FAULT0 A, FAULT1 A,
                                                        *     OVF, TRG, UFS A */
#define SAM_IRQ_TCC0MC0        (SAM_IRQ_EXTINT + 86)   /* 86  TCC0: MC 0 */
#define SAM_IRQ_TCC0MC1        (SAM_IRQ_EXTINT + 87)   /* 87  TCC0: MC 1 */
#define SAM_IRQ_TCC0MC2        (SAM_IRQ_EXTINT + 88)   /* 88  TCC0: MC 2 */
#define SAM_IRQ_TCC0MC3        (SAM_IRQ_EXTINT + 89)   /* 89  TCC0: MC 3 */
#define SAM_IRQ_TCC0MC4        (SAM_IRQ_EXTINT + 90)   /* 90  TCC0: MC 4 */
#define SAM_IRQ_TCC0MC5        (SAM_IRQ_EXTINT + 91)   /* 91  TCC0: MC 5 */
#define SAM_IRQ_TCC1           (SAM_IRQ_EXTINT + 92)   /* 92  TCC1: CNT A, DFS A, ERR A, FAULTA
                                                        *     A, FAULTB A, FAULT0 A, FAULT1 A,
                                                        *     OVF, TRG, UFS A */
#define SAM_IRQ_TCC1MC0        (SAM_IRQ_EXTINT + 93)   /* 93  TCC1: MC 0 */
#define SAM_IRQ_TCC1MC1        (SAM_IRQ_EXTINT + 94)   /* 94  TCC1: MC 1 */
#define SAM_IRQ_TCC1MC2        (SAM_IRQ_EXTINT + 95)   /* 95  TCC1: MC 2 */
#define SAM_IRQ_TCC1MC3        (SAM_IRQ_EXTINT + 96)   /* 96  TCC1: MC 3 */
#define SAM_IRQ_TCC2           (SAM_IRQ_EXTINT + 97)   /* 97  TCC2: CNT A, DFS A, ERR A, FAULTA
                                                        *     A, FAULTB A, FAULT0 A, FAULT1 A,
                                                        *     OVF, TRG, UFS A */
#define SAM_IRQ_TCC2MC0        (SAM_IRQ_EXTINT + 98)   /* 98  TCC2: MC 0 */
#define SAM_IRQ_TCC2MC1        (SAM_IRQ_EXTINT + 99)   /* 99  TCC2: MC 1 */
#define SAM_IRQ_TCC2MC2        (SAM_IRQ_EXTINT + 100)  /* 100 TCC2: MC 2 */
#define SAM_IRQ_TCC3           (SAM_IRQ_EXTINT + 101)  /* 101 TCC3: CNT A, DFS A, ERR A, FAULTA
                                                        *     A, FAULTB A, FAULT0 A, FAULT1 A,
                                                        *     OVF, TRG, UFS A */
#define SAM_IRQ_TCC3MC0        (SAM_IRQ_EXTINT + 102)  /* 102 TCC3: MC 0 */
#define SAM_IRQ_TCC3MC1        (SAM_IRQ_EXTINT + 103)  /* 103 TCC3: MC 1 */
#define SAM_IRQ_TCC4           (SAM_IRQ_EXTINT + 104)  /* 104 TCC4: CNT A, DFS A, ERR A, FAULTA
                                                        *     A, FAULTB A, FAULT0 A, FAULT1 A,
                                                        *     OVF, TRG, UFS A */
#define SAM_IRQ_TCC4MC0        (SAM_IRQ_EXTINT + 105)  /* 105 TCC4: MC 0 */
#define SAM_IRQ_TCC4MC1        (SAM_IRQ_EXTINT + 106)  /* 106 TCC4: MC 1 */
#define SAM_IRQ_TC0            (SAM_IRQ_EXTINT + 107)  /* 107 TC0: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC1            (SAM_IRQ_EXTINT + 108)  /* 108 TC1: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC2            (SAM_IRQ_EXTINT + 109)  /* 109 TC2: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC3            (SAM_IRQ_EXTINT + 110)  /* 110 TC3: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC4            (SAM_IRQ_EXTINT + 111)  /* 111 TC4: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC5            (SAM_IRQ_EXTINT + 112)  /* 112 TC5: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC6            (SAM_IRQ_EXTINT + 113)  /* 113 TC6: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC7            (SAM_IRQ_EXTINT + 114)  /* 114 TC7: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_PDEC           (SAM_IRQ_EXTINT + 115)  /* 115 PDEC: DIR A, ERR A, OVF, VLC A */
#define SAM_IRQ_PDECMC0        (SAM_IRQ_EXTINT + 116)  /* 116 PDEC: MC 0 */
#define SAM_IRQ_PDECMC1        (SAM_IRQ_EXTINT + 117)  /* 117 PDEC: MC 1 */
#define SAM_IRQ_ADC0           (SAM_IRQ_EXTINT + 118)  /* 118 ADC0: OVERRUN, WINMON */
#define SAM_IRQ_ADC0RDY        (SAM_IRQ_EXTINT + 119)  /* 119 ADC0: RESRDY */
#define SAM_IRQ_ADC1           (SAM_IRQ_EXTINT + 120)  /* 120 ADC0: OVERRUN, WINMON */
#define SAM_IRQ_ADC1RDY        (SAM_IRQ_EXTINT + 121)  /* 121 ADC0: RESRDY */
#define SAM_IRQ_AC             (SAM_IRQ_EXTINT + 122)  /* 122 AC: COMP 0, COMP 1, WIN 0 */
#define SAM_IRQ_DACERR         (SAM_IRQ_EXTINT + 123)  /* 123 DAC: OVERRUN A 0, OVERRUN A 1,
                                                        *     UNDERRUN A 0, UNDERRUN A 1 */
#define SAM_IRQ_DACEMPTY0      (SAM_IRQ_EXTINT + 124)  /* 124 DAC: EMPTY 0 */
#define SAM_IRQ_DACEMPTY1      (SAM_IRQ_EXTINT + 125)  /* 125 DAC: EMPTY 1 */
#define SAM_IRQ_DACRDY0        (SAM_IRQ_EXTINT + 126)  /* 126 DAC: RESRDY 0 */
#define SAM_IRQ_DACRDY1        (SAM_IRQ_EXTINT + 127)  /* 127 DAC: RESRDY 1 */
#define SAM_IRQ_I2S            (SAM_IRQ_EXTINT + 128)  /* 128 I2S: RXOR 0, RXOR 1, RXRDY 0, RXRDY
                                                        *     1, TXRDY 0, TXRDY 1, TXUR 0, TXUR 1 */
#define SAM_IRQ_PCC            (SAM_IRQ_EXTINT + 129)  /* 129 PCC: */
#define SAM_IRQ_AES            (SAM_IRQ_EXTINT + 130)  /* 130 AES: ENCCMP, GFMCMP */
#define SAM_IRQ_TRNG           (SAM_IRQ_EXTINT + 131)  /* 131 TRNG: IS0 */
#define SAM_IRQ_ICM            (SAM_IRQ_EXTINT + 132)  /* 132 ICM: */
#define SAM_IRQ_PUKCC          (SAM_IRQ_EXTINT + 133)  /* 133 PUKCC:  */
#define SAM_IRQ_QSPI           (SAM_IRQ_EXTINT + 134)  /* 134 QSPI:  */
#define SAM_IRQ_SDHC0          (SAM_IRQ_EXTINT + 135)  /* 135 SDHC0: SDHC0, TIMER */
#define SAM_IRQ_SDHC1          (SAM_IRQ_EXTINT + 136)  /* 136 SDHC1: SDHC1, TIMER */

#define SAM_IRQ_NEXTINT        137                     /* Total number of external interrupt numbers */

#define NR_IRQS                (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT) /* The number of vectors */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_SAMD5E5_SAMD5E5_IRQ_H */
