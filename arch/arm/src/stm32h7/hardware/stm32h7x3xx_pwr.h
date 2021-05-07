/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32h7x3xx_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_PWR_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_PWR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_PWR_CR1_OFFSET        0x0000  /* Power control register 1 */
#define STM32_PWR_CSR1_OFFSET       0x0004  /* Power control/status register 1 */
#define STM32_PWR_CR2_OFFSET        0x0008  /* Power control register 2 */
#define STM32_PWR_CR3_OFFSET        0x000c  /* Power control register 3 */
#define STM32_PWR_CPUCR_OFFSET      0x0010  /* Power CPU control register */
                                            /* 0x014: Reserved */
#define STM32_PWR_D3CR_OFFSET       0x0018  /* Power D3 domain control register */
#define STM32_PWR_WKUPCR_OFFSET     0x0020  /* Power wakeup clear register */
#define STM32_PWR_WKUPFR_OFFSET     0x0024  /* Power wakeup flag register */
#define STM32_PWR_WKUPEPR_OFFSET    0x0028  /* Power wakeup enable and polarity register*/
                                            /* 0x030: Reserved */

/* Register Addresses *******************************************************/

#define STM32_PWR_CR1               (STM32_PWR_BASE+STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CSR1              (STM32_PWR_BASE+STM32_PWR_CSR1_OFFSET)
#define STM32_PWR_CR2               (STM32_PWR_BASE+STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CR3               (STM32_PWR_BASE+STM32_PWR_CR3_OFFSET)
#define STM32_PWR_CPUCR             (STM32_PWR_BASE+STM32_PWR_CPUCR_OFFSET)
#define STM32_PWR_D3CR              (STM32_PWR_BASE+STM32_PWR_D3CR_OFFSET)
#define STM32_PWR_WKUPCR            (STM32_PWR_BASE+STM32_PWR_WKUPCR_OFFSET)
#define STM32_PWR_WKUPFR            (STM32_PWR_BASE+STM32_PWR_WKUPFR_OFFSET)
#define STM32_PWR_WKUPEOR           (STM32_PWR_BASE+STM32_PWR_WKUPEOR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Power control register 1 (CR1) */

#define PWR_CR1_LPDS                (1 << 0)    /* Bit 0: Low-power Deepsleep with SVOS3 */
                                                /* Bits 1-3: Reserved */
#define PWR_CR1_PVDE                (1 << 4)    /* Bit 4: Programmable voltage detector enable */

#define PWR_CR1_PLS_SHIFT           (5)        /* Bits 5-7: Programmable voltage detector level */
#define PWR_CR1_PLS_MASK            (7 << PWR_CR1_PLS_SHIFT)
#  define PWR_CR1_PLS_1V95          (0 << PWR_CR1_PLS_SHIFT) /* 000: */
#  define PWR_CR1_PLS_2V1           (1 << PWR_CR1_PLS_SHIFT) /* 001: */
#  define PWR_CR1_PLS_2V25          (2 << PWR_CR1_PLS_SHIFT) /* 010: */
#  define PWR_CR1_PLS_2V4           (3 << PWR_CR1_PLS_SHIFT) /* 011: */
#  define PWR_CR1_PLS_2V55          (4 << PWR_CR1_PLS_SHIFT) /* 100: */
#  define PWR_CR1_PLS_2V7           (5 << PWR_CR1_PLS_SHIFT) /* 101: */
#  define PWR_CR1_PLS_2V85          (6 << PWR_CR1_PLS_SHIFT) /* 110: */
#  define PWR_CR1_PLS_EXT           (7 << PWR_CR1_PLS_SHIFT) /* 111: */

#define PWR_CR1_DBP                 (1 << 8)   /* Bit 8: Disable backup domain write protection */
#define PWR_CR1_FLPS                (1 << 9)   /* Bit 9: */
                                               /* Bits 10-13: Reserved */
#define PWR_CR1_SVOS_SHIFT          (14)       /* Bits 14-15: */
#define PWR_CR1_SVOS_MASK           (3 << PWR_CR1_SVOS_SHIFT)
                                                              /* 00: Reserved */
#  define PWR_CR1_SVOS_S5           (1 << PWR_CR1_SVOS_SHIFT) /* 01:  */
#  define PWR_CR1_SVOS_S4           (2 << PWR_CR1_SVOS_SHIFT) /* 10: */
#  define PWR_CR1_SVOS_S3           (3 << PWR_CR1_SVOS_SHIFT) /* 11: */

#define PWR_CR1_AVDEN               (1 << 16)  /* Bit 16: */
#define PWR_CR1_ALS_SHIFT           (17)       /* Bits 17-18: Analog voltage detector level selection */
#define PWR_CR1_ALS_MASK            (3 << PWR_CR1_ALS_SHIFT)
#  define PWR_CR1_ALS_1V7           (0 << PWR_CR1_ALS_SHIFT) /* 00: */
#  define PWR_CR1_ALS_2V1           (1 << PWR_CR1_ALS_SHIFT) /* 01 */
#  define PWR_CR1_ALS_2V5           (2 << PWR_CR1_ALS_SHIFT) /* 10: */
#  define PWR_CR1_ALS_2V8           (3 << PWR_CR1_ALS_SHIFT) /* 11: */

                                              /* Bits 19-31: Reserved */

/* Power control/status register 1 (CSR1) */

                                              /* Bits 0-3: Reserved */
#define PWR_CSR1_PVDO               (1 << 4)  /* Bit 4: Programmable voltage detect output*/
                                              /* Bits 5-12: Reserved */
#define PWR_CSR1_ACTVOSRDY          (1 << 13) /* Bit 13: VOS voltage level ready */
#define PWR_CSR1_ACTVOS_SHIFT       (14)      /* Bits 14-15: Current VOS applied */
#  define PWR_CSR1_VOS_SCALE_3R     (0 << PWR_CSR1_ACTVOS_SHIFT)
#  define PWR_CSR1_VOS_SCALE_3      (1 << PWR_CSR1_ACTVOS_SHIFT)
#  define PWR_CSR1_VOS_SCALE_2      (2 << PWR_CSR1_ACTVOS_SHIFT)
#  define PWR_CSR1_VOS_SCALE_1      (3 << PWR_CSR1_ACTVOS_SHIFT)
#  define PWR_CSR1_VOS_SCALE_0      (3 << PWR_CSR1_ACTVOS_SHIFT)
#define PWR_CSR1_AVDO               (1 << 16) /* Bit 16: Analog voltage detector output */
                                              /* Bits 17-31: Reserved */

/* Power control register 2 (CR2) */

#define PWR_CR2_BREN                (1 << 0)   /* Bit 0: Backup regulator enable */
                                               /* Bits 1-3: Reserved */
#define PWR_CR2_MONEN               (1 << 4)   /* Bit 4: VBAT and temperature monitoring enable */
                                               /* Bits 5-15: Reserved */
#define PWR_CR2_BRRDY               (1 << 16)  /* Bit 16: Backup regulator ready */
                                               /* Bits 17-19: Reserved */
#define PWR_CR2_VBATL               (1 << 20)  /* Bit 20: VBAT level monitoring versus low threshold */
#define PWR_CR2_VBATH               (1 << 21)  /* Bit 21: VBAT level monitoring versus high threshold */
#define PWR_CR2_TEMPL               (1 << 22)  /* Bit 22: Temperature level monitoring versus low threshold */
#define PWR_CR2_TEMPH               (1 << 23)  /* Bit 23: Temperature level monitoring versus high threshold */
                                               /* Bits 24-31: Reserved */

/* Power control register 3 (CR3) */

#define STM32_PWR_CR3_BYPASS        (1 << 0)  /* Bit 0: Power management unit bypass */
#define STM32_PWR_CR3_LDOEN         (1 << 1)  /* Bit 1: Low drop-out regulator enable */
#define STM32_PWR_CR3_LDOESCUEN     (1 << 2)  /* Bit 2: Supply configuration update enable */
                                              /* Bits 3-7: Reserved */
#define STM32_PWR_CR3_VBE           (1 << 8)  /* Bit 8: VBAT charging enable */
#define STM32_PWR_CR3_VBRS          (1 << 9)  /* Bit 9: VBAT charging resistor selection */
                                              /* Bits 10-23: Reserved */
#define STM32_PWR_CR3_USB33DEN      (1 << 24) /* Bit 24: VDD33USB voltage level detector enable */
#define STM32_PWR_CR3_USBREGEN      (1 << 25) /* Bit 25: USB regulator enable */
#define STM32_PWR_CR3_USB33RDY      (1 << 26) /* Bit 26: USB supply ready */

/* Power CPU control register (CPUCR) */

#define STM32_PWR_CPUCR_PDDS_D1     (1 << 0)  /* Bit 0: D1 domain Power Down Deepsleep selection */
#define STM32_PWR_CPUCR_PDDS_D2     (1 << 1)  /* Bit 1: D2 domain Power Down Deepsleep */
#define STM32_PWR_CPUCR_PDDS_D3     (1 << 2)  /* Bit 2: System D3 domain Power Down Deepsleep */
                                              /* Bits 3-4: Reserved */
#define STM32_PWR_CPUCR_STOPF       (1 << 5)  /* Bit 5: STOP flag */
#define STM32_PWR_CPUCR_SBF         (1 << 6)  /* Bit 6: System Standby flag */
#define STM32_PWR_CPUCR_SBF_D1      (1 << 7)  /* Bit 7: D1 domain DStandby flag */
#define STM32_PWR_CPUCR_SBF_D2      (1 << 8)  /* Bit 8: D2 domain DStandby flag */
#define STM32_PWR_CPUCR_CSSF        (1 << 9)  /* Bit 9: Clear Standby and Stop flags (always read as 0) */
                                              /* Bit 10: Reserved */
#define STM32_PWR_CPUCR_RUN_D3      (1 << 11) /* Bit 11: Keep system D3 domain in Run mode regardless of the CPU subsystem modes */
                                              /* Bits 12-31: Reserved */

/* Power D3 domain control register (D3CR) */

                                              /* Bits 0-12: Reserved */
#define STM32_PWR_D3CR_VOSRDY       (1 << 13) /* Bit 13:  */
#define STM32_PWR_D3CR_VOS_SHIFT    (14)      /* Bits 14-15: Voltage scaling selection according to performance */
#define STM32_PWR_D3CR_VOS_MASK     (3 << STM32_PWR_D3CR_VOS_SHIFT)
#  define PWR_D3CR_VOS_SCALE_3R     (0 << STM32_PWR_D3CR_VOS_SHIFT) /* Fmax = 200MHz */
#  define PWR_D3CR_VOS_SCALE_3      (1 << STM32_PWR_D3CR_VOS_SHIFT) /* Fmax = 200MHz */
#  define PWR_D3CR_VOS_SCALE_2      (2 << STM32_PWR_D3CR_VOS_SHIFT) /* Fmax = 300MHz */
#  define PWR_D3CR_VOS_SCALE_1      (3 << STM32_PWR_D3CR_VOS_SHIFT) /* Fmax = 400MHz */
#  define PWR_D3CR_VOS_SCALE_0      (3 << STM32_PWR_D3CR_VOS_SHIFT) /* Fmax = 480MHz with ODN */

                                              /* Bits 15-31: Reserved */

/* Power wakeup clear register (WKUPCR) */

#define STM32_PWR_WKUPC1            (1 << 0)  /* Bit 0: Clear wakeup pin flag for WKUP1 */
#define STM32_PWR_WKUPC2            (1 << 1)  /* Bit 1: Clear wakeup pin flag for WKUP2 */
#define STM32_PWR_WKUPC3            (1 << 2)  /* Bit 2: Clear wakeup pin flag for WKUP3 */
#define STM32_PWR_WKUPC4            (1 << 3)  /* Bit 3: Clear wakeup pin flag for WKUP4 */
#define STM32_PWR_WKUPC5            (1 << 4)  /* Bit 4: Clear wakeup pin flag for WKUP5 */
#define STM32_PWR_WKUPC6            (1 << 5)  /* Bit 5: Clear wakeup pin flag for WKUP6 */
                                              /* Bits 6-31: Reserved */

/* Power wakeup flag register (WKUPFR) */

#define STM32_PWR_WKUPF1            (1 << 0)  /* Bit 0: Wakeup pin flag for WKUP1 */
#define STM32_PWR_WKUPF2            (1 << 1)  /* Bit 1: Wakeup pin flag for WKUP2 */
#define STM32_PWR_WKUPF3            (1 << 2)  /* Bit 2: Wakeup pin flag for WKUP3 */
#define STM32_PWR_WKUPF4            (1 << 3)  /* Bit 3: Wakeup pin flag for WKUP4 */
#define STM32_PWR_WKUPF5            (1 << 4)  /* Bit 4: Wakeup pin flag for WKUP5 */
#define STM32_PWR_WKUPF6            (1 << 5)  /* Bit 5: Wakeup pin flag for WKUP6 */
                                              /* Bits 6-31: Reserved */

/* Power wakeup enable and polarity register (WKUPEPR) */

#define STM32_PWR_WKUPEN(n)         (1 << (n))
#define STM32_PWR_WKUPEN1           (1 << 0)  /* Bit 0: Enable wakeup pin WKUP1 */
#define STM32_PWR_WKUPEN2           (1 << 1)  /* Bit 1: Enable wakeup pin WKUP2 */
#define STM32_PWR_WKUPEN3           (1 << 2)  /* Bit 2: Enable wakeup pin WKUP3 */
#define STM32_PWR_WKUPEN4           (1 << 3)  /* Bit 3: Enable wakeup pin WKUP4 */
#define STM32_PWR_WKUPEN5           (1 << 4)  /* Bit 4: Enable wakeup pin WKUP5 */
#define STM32_PWR_WKUPEN6           (1 << 5)  /* Bit 5: Enable wakeup pin WKUP6 */
                                              /* Bits 6-7: Reserved */
#define STM32_PWR_WKUPP(n)          (1 << (n + 8))
#define STM32_PWR_WKUPP1            (1 << 8)  /* Bit 8: Wakeup pin polarity for WKUP1 */
#define STM32_PWR_WKUPP2            (1 << 9)  /* Bit 9: Wakeup pin polarity for WKUP2 */
#define STM32_PWR_WKUPP3            (1 << 10) /* Bit 10: Wakeup pin polarity for WKUP3 */
#define STM32_PWR_WKUPP4            (1 << 11) /* Bit 11: Wakeup pin polarity for WKUP4 */
#define STM32_PWR_WKUPP5            (1 << 12) /* Bit 12: Wakeup pin polarity for WKUP5 */
#define STM32_PWR_WKUPP6            (1 << 13) /* Bit 13: Wakeup pin polarity for WKUP6 */
                                              /* Bits 14-15: Reserved */
#define STM32_PWR_WKUPPUPD_SHIFT(n) (n * 2 + 16)
#define STM32_PWR_WKUPPUPD1_SHIFT   (16)      /* Bits 16-17: Wakeup pin pull config for WKUP1 */
#define STM32_PWR_WKUPPUPD2_SHIFT   (18)      /* Bits 18-19: Wakeup pin pull config for WKUP2 */
#define STM32_PWR_WKUPPUPD3_SHIFT   (20)      /* Bits 20-21: Wakeup pin pull config for WKUP3 */
#define STM32_PWR_WKUPPUPD4_SHIFT   (22)      /* Bits 22-23: Wakeup pin pull config for WKUP4 */
#define STM32_PWR_WKUPPUPD5_SHIFT   (24)      /* Bits 24-25: Wakeup pin pull config for WKUP5 */
#define STM32_PWR_WKUPPUPD6_SHIFT   (26)      /* Bits 26-27: Wakeup pin pull config for WKUP6 */

# define STM32_PWR_WKUPPUPD_NONE    (0)      /* No pull-up */
# define STM32_PWR_WKUPPUPD_PULLUP  (1)      /* Pull-up enabled */
# define STM32_PWR_WKUPPUPD_PULLDN  (2)      /* Pull-down enabled */
                                             /* 3 is reserved */
# define STM32_PWR_WKUPPUPD_MASK    (3)
                                             /* Bits 28-31: Reserved */

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_PWR_H */
