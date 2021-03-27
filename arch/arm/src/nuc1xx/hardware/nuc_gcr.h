/****************************************************************************
 * arch/arm/src/nuc1xx/hardware/nuc_gcr.h
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

#ifndef __ARCH_ARM_SRC_NUC1XX_HARDWARE_NUC_GCR_H
#define __ARCH_ARM_SRC_NUC1XX_HARDWARE_NUC_GCR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NUC_GCR_PDID_OFFSET      0x0000  /* Part didentification number register */
#define NUC_GCR_RSTSRC_OFFSET    0x0004  /* System reset source register */
#define NUC_GCR_IPRSTC1_OFFSET   0x0008  /* IP Reset control register 1 */
#define NUC_GCR_IPRSTC2_OFFSET   0x000c  /* IP Reset control register 2 */
#define NUC_GCR_CPR_OFFSET       0x0010  /* Chip performance register */
#define NUC_GCR_BODCR_OFFSET     0x0018  /* Brown-out detector control register */
#define NUC_GCR_TEMPCR_OFFSET    0x001c  /* Temperature sensor control register */
#define NUC_GCR_PORCR_OFFSET     0x0024  /* Power-on-reset control register */
#define NUC_GCR_GPA_MFP_OFFSET   0x0030  /* Multiple function pin GPIOA control register */
#define NUC_GCR_GPB_MFP_OFFSET   0x0034  /* Multiple function pin GPIOB control register */
#define NUC_GCR_GPC_MFP_OFFSET   0x0038  /* Multiple function pin GPIOC control register */
#define NUC_GCR_GPD_MFP_OFFSET   0x003C  /* Multiple function pin GPIOD control register */
#define NUC_GCR_GPE_MFP_OFFSET   0x0040  /* Multiple function pin GPIOE control register */
#define NUC_GCR_ALT_MFP_OFFSET   0x0050  /* Alternative multiple function pin control register */
#define NUC_GCR_REGWRPROT_OFFSET 0x0100  /* Register write-protection control register */

/* Register addresses *******************************************************/

#define NUC_GCR_PDID             (NUC_GCR_BASE+NUC_GCR_PDID_OFFSET)
#define NUC_GCR_RSTSRC           (NUC_GCR_BASE+NUC_GCR_RSTSRC_OFFSET)
#define NUC_GCR_IPRSTC1          (NUC_GCR_BASE+NUC_GCR_IPRSTC1_OFFSET)
#define NUC_GCR_IPRSTC2          (NUC_GCR_BASE+NUC_GCR_IPRSTC2_OFFSET)
#define NUC_GCR_CPR              (NUC_GCR_BASE+NUC_GCR_CPR_OFFSET)
#define NUC_GCR_BODCR            (NUC_GCR_BASE+NUC_GCR_BODCR_OFFSET)
#define NUC_GCR_TEMPCR           (NUC_GCR_BASE+NUC_GCR_TEMPCR_OFFSET)
#define NUC_GCR_PORCR            (NUC_GCR_BASE+NUC_GCR_PORCR_OFFSET)
#define NUC_GCR_GPA_MFP          (NUC_GCR_BASE+NUC_GCR_GPA_MFP_OFFSET)
#define NUC_GCR_GPB_MFP          (NUC_GCR_BASE+NUC_GCR_GPB_MFP_OFFSET)
#define NUC_GCR_GPC_MFP          (NUC_GCR_BASE+NUC_GCR_GPC_MFP_OFFSET)
#define NUC_GCR_GPD_MFP          (NUC_GCR_BASE+NUC_GCR_GPD_MFP_OFFSET)
#define NUC_GCR_GPE_MFP          (NUC_GCR_BASE+NUC_GCR_GPE_MFP_OFFSET)
#define NUC_GCR_ALT_MFP          (NUC_GCR_BASE+NUC_GCR_ALT_MFP_OFFSET)
#define NUC_GCR_REGWRPROT        (NUC_GCR_BASE+NUC_GCR_REGWRPROT_OFFSET)

/* Register bit-field definitions *******************************************/

/* Part didentification number register (32-bit part ID number) */

/* System reset source register */

#define GCR_RSTSRC_POR           (1 << 0)  /* Bit 0:  Power-on reset (POR) or chip reset (CHIP_RST) */
#define GCR_RSTSRC_RESET         (1 << 1)  /* Bit 1:  /RESET pin */
#define GCR_RSTSRC_WDT           (1 << 2)  /* Bit 2:  Watchdog timer */
#define GCR_RSTSRC_LVR           (1 << 3)  /* Bit 3:  Low voltage reset controller */
#define GCR_RSTSRC_BOD           (1 << 4)  /* Bit 4:  Brown-out detection */
#define GCR_RSTSRC_SYS           (1 << 5)  /* Bit 5:  Software set AIRCR:SYSRESETREQ */
#define GCR_RSTSRC_CPU           (1 << 7)  /* Bit 7:  Software set CPU_RST */

/* IP Reset control register 1 */

#define GCR_IPRSTC1_CHIP_RST     (1 << 0)  /* Bit 0:  Chip one-shot reset */
#define GCR_IPRSTC1_CPU_RST      (1 << 1)  /* Bit 1:  CPU kernel one-shot reset */
#define GCR_IPRSTC1_PDMA_RST     (1 << 2)  /* Bit 2:  PDMA controller reset */
#define GCR_IPRSTC1_EBI_RST      (1 << 3)  /* Bit 3:  EBI controller reset */

/* IP Reset control register 2 */

#define GCR_IPRSTC2_GPIO_RST     (1 << 1)  /* Bit 1:  GPIO controller reset */
#define GCR_IPRSTC2_TMR0_RST     (1 << 2)  /* Bit 2:  Timer0 controller reset */
#define GCR_IPRSTC2_TMR1_RST     (1 << 3)  /* Bit 3:  Timer1 controller reset */
#define GCR_IPRSTC2_TMR2_RST     (1 << 4)  /* Bit 4:  Timer2 controller reset */
#define GCR_IPRSTC2_TMR3_RST     (1 << 5)  /* Bit 5:  Timer3 controller reset */
#define GCR_IPRSTC2_I2C0_RST     (1 << 8)  /* Bit 8:  I2C0 controller reset */
#define GCR_IPRSTC2_I2C1_RST     (1 << 9)  /* Bit 9:  I2C1 controller reset */
#define GCR_IPRSTC2_SPI0_RST     (1 << 12) /* Bit 12: SPI0 controller reset */
#define GCR_IPRSTC2_SPI1_RST     (1 << 13) /* Bit 13: SPI1 controller reset */
#define GCR_IPRSTC2_SPI2_RST     (1 << 14) /* Bit 14: SPI2 controller reset */
#define GCR_IPRSTC2_SPI3_RST     (1 << 15) /* Bit 15: SPI3 controller reset */
#define GCR_IPRSTC2_UART0_RST    (1 << 16) /* Bit 16: UART0 controller reset */
#define GCR_IPRSTC2_UART1_RST    (1 << 17) /* Bit 17: UART1 controller reset */
#define GCR_IPRSTC2_UART2_RST    (1 << 18) /* Bit 18: UART2 controller reset */
#define GCR_IPRSTC2_PWM03_RST    (1 << 20) /* Bit 20: PWM0/1/2/3 controller reset */
#define GCR_IPRSTC2_PWM47_RST    (1 << 21) /* Bit 21: PWM4/5/6/7 controller reset */
#define GCR_IPRSTC2_ACMP_RST     (1 << 22) /* Bit 22: Analog comparator controller reset */
#define GCR_IPRSTC2_PS2_RST      (1 << 23) /* Bit 23: PS/2 controller reset */
#define GCR_IPRSTC2_USBD_RST     (1 << 27) /* Bit 27: USB device controller reset */
#define GCR_IPRSTC2_ADC_RST      (1 << 28) /* Bit 28: ADC controller reset */
#define GCR_IPRSTC2_I2S_RST      (1 << 29) /* Bit 29: I2S controller reset */

/* Chip performance register */

#define GCR_CPR_HPE              (1 << 0)  /* Bit 0:  High performance enable */

/* Brown-out detector control register */

#define GCR_BODCR_BOD_EN         (1 << 0)  /* Bit 0:  Brown-ut detector enable */
#define GCR_BODCR_BOD_VL_SHIFT   (1)       /* Bits 1-2: Brown-out detector threshold voltage selection */
#define GCR_BODCR_BOD_VL_MASK    (3 << GCR_BODCR_BOD_VL_SHIFT)
#  define GCR_BODCR_BOD_VL_2p2V  (0 << GCR_BODCR_BOD_VL_SHIFT) /* 2.2V */
#  define GCR_BODCR_BOD_VL_2p7V  (1 << GCR_BODCR_BOD_VL_SHIFT) /* 2.7V */
#  define GCR_BODCR_BOD_VL_3p8V  (2 << GCR_BODCR_BOD_VL_SHIFT) /* 3.8V */
#  define GCR_BODCR_BOD_VL_4p5V  (3 << GCR_BODCR_BOD_VL_SHIFT) /* 4.5V */

#define GCR_BODCR_BOD_RSTEN      (1 << 3)  /* Bit 3:  Brown-out reset enable */
#define GCR_BODCR_BOD_INTF       (1 << 4)  /* Bit 4:  Brown-out deletector interrupt flag */
#define GCR_BODCR_BOD_LPM        (1 << 5)  /* Bit 5:  Brown-out detector low power mode */
#define GCR_BODCR_BOD_OUT        (1 << 6)  /* Bit 6:  Brown-out detector output status */
#define GCR_BODCR_LVR_EN         (1 << 7)  /* Bit 7:  Low voltaghe reset enable */

/* Temperature sensor control register */

#define GCR_TEMPCR_VTEMP_EN      (1 << 0)  /* Bit 0:  Temperature sensor enable */

/* Power-on-reset control register */

#define GCR_PORCR_MASK           (0xffff)  /* Bits 9-15: POR disable code */

/* Multiple function pin GPIOA control register */

#define GCR_GPA_MFP(n)           (1 << (n)) /* Bits 0-15: PAn pin function selection */
#  define GCR_GPA_MFP0           (1 << 0)
#  define GCR_GPA_MFP1           (1 << 1)
#  define GCR_GPA_MFP2           (1 << 2)
#  define GCR_GPA_MFP3           (1 << 3)
#  define GCR_GPA_MFP4           (1 << 4)
#  define GCR_GPA_MFP5           (1 << 5)
#  define GCR_GPA_MFP6           (1 << 6)
#  define GCR_GPA_MFP7           (1 << 7)
#  define GCR_GPA_MFP8           (1 << 8)
#  define GCR_GPA_MFP9           (1 << 9)
#  define GCR_GPA_MFP10          (1 << 10)
#  define GCR_GPA_MFP11          (1 << 11)
#  define GCR_GPA_MFP12          (1 << 12)
#  define GCR_GPA_MFP13          (1 << 13)
#  define GCR_GPA_MFP14          (1 << 14)
#  define GCR_GPA_MFP15          (1 << 15)
#define GCR_GPA_TYPE(n)          (1 << ((n)+16)) /* Bits 16-31: Enable Schmitt trigger function */
#  define GCR_GPA_TYPE0          (1 << 0)
#  define GCR_GPA_TYPE1          (1 << 1)
#  define GCR_GPA_TYPE2          (1 << 2)
#  define GCR_GPA_TYPE3          (1 << 3)
#  define GCR_GPA_TYPE4          (1 << 4)
#  define GCR_GPA_TYPE5          (1 << 5)
#  define GCR_GPA_TYPE6          (1 << 6)
#  define GCR_GPA_TYPE7          (1 << 7)
#  define GCR_GPA_TYPE8          (1 << 8)
#  define GCR_GPA_TYPE9          (1 << 9)
#  define GCR_GPA_TYPE10         (1 << 10)
#  define GCR_GPA_TYPE11         (1 << 11)
#  define GCR_GPA_TYPE12         (1 << 12)
#  define GCR_GPA_TYPE13         (1 << 13)
#  define GCR_GPA_TYPE14         (1 << 14)
#  define GCR_GPA_TYPE15         (1 << 15)

/* Multiple function pin GPIOB control register */

#define GCR_GPB_MFP(n)           (1 << (n)) /* Bits 0-15: PBn pin function selection */
#  define GCR_GPB_MFP0           (1 << 0)
#  define GCR_GPB_MFP1           (1 << 1)
#  define GCR_GPB_MFP2           (1 << 2)
#  define GCR_GPB_MFP3           (1 << 3)
#  define GCR_GPB_MFP4           (1 << 4)
#  define GCR_GPB_MFP5           (1 << 5)
#  define GCR_GPB_MFP6           (1 << 6)
#  define GCR_GPB_MFP7           (1 << 7)
#  define GCR_GPB_MFP8           (1 << 8)
#  define GCR_GPB_MFP9           (1 << 9)
#  define GCR_GPB_MFP10          (1 << 10)
#  define GCR_GPB_MFP11          (1 << 11)
#  define GCR_GPB_MFP12          (1 << 12)
#  define GCR_GPB_MFP13          (1 << 13)
#  define GCR_GPB_MFP14          (1 << 14)
#  define GCR_GPB_MFP15          (1 << 15)
#define GCR_GPB_TYPE(n)          (1 << ((n)+16)) /* Bits 16-31: Enable Schmitt trigger function */
#  define GCR_GPB_TYPE0          (1 << 0)
#  define GCR_GPB_TYPE1          (1 << 1)
#  define GCR_GPB_TYPE2          (1 << 2)
#  define GCR_GPB_TYPE3          (1 << 3)
#  define GCR_GPB_TYPE4          (1 << 4)
#  define GCR_GPB_TYPE5          (1 << 5)
#  define GCR_GPB_TYPE6          (1 << 6)
#  define GCR_GPB_TYPE7          (1 << 7)
#  define GCR_GPB_TYPE8          (1 << 8)
#  define GCR_GPB_TYPE9          (1 << 9)
#  define GCR_GPB_TYPE10         (1 << 10)
#  define GCR_GPB_TYPE11         (1 << 11)
#  define GCR_GPB_TYPE12         (1 << 12)
#  define GCR_GPB_TYPE13         (1 << 13)
#  define GCR_GPB_TYPE14         (1 << 14)
#  define GCR_GPB_TYPE15         (1 << 15)

/* Multiple function pin GPIOC control register */

#define GCR_GPC_MFP(n)           (1 << (n)) /* Bits 0-15: PCn pin function selection */
#  define GCR_GPC_MFP0           (1 << 0)
#  define GCR_GPC_MFP1           (1 << 1)
#  define GCR_GPC_MFP2           (1 << 2)
#  define GCR_GPC_MFP3           (1 << 3)
#  define GCR_GPC_MFP4           (1 << 4)
#  define GCR_GPC_MFP5           (1 << 5)
#  define GCR_GPC_MFP6           (1 << 6)
#  define GCR_GPC_MFP7           (1 << 7)
#  define GCR_GPC_MFP8           (1 << 8)
#  define GCR_GPC_MFP9           (1 << 9)
#  define GCR_GPC_MFP10          (1 << 10)
#  define GCR_GPC_MFP11          (1 << 11)
#  define GCR_GPC_MFP12          (1 << 12)
#  define GCR_GPC_MFP13          (1 << 13)
#  define GCR_GPC_MFP14          (1 << 14)
#  define GCR_GPC_MFP15          (1 << 15)
#define GCR_GPC_TYPE(n)          (1 << ((n)+16)) /* Bits 16-31: Enable Schmitt trigger function */
#  define GCR_GPC_TYPE0          (1 << 0)
#  define GCR_GPC_TYPE1          (1 << 1)
#  define GCR_GPC_TYPE2          (1 << 2)
#  define GCR_GPC_TYPE3          (1 << 3)
#  define GCR_GPC_TYPE4          (1 << 4)
#  define GCR_GPC_TYPE5          (1 << 5)
#  define GCR_GPC_TYPE6          (1 << 6)
#  define GCR_GPC_TYPE7          (1 << 7)
#  define GCR_GPC_TYPE8          (1 << 8)
#  define GCR_GPC_TYPE9          (1 << 9)
#  define GCR_GPC_TYPE10         (1 << 10)
#  define GCR_GPC_TYPE11         (1 << 11)
#  define GCR_GPC_TYPE12         (1 << 12)
#  define GCR_GPC_TYPE13         (1 << 13)
#  define GCR_GPC_TYPE14         (1 << 14)
#  define GCR_GPC_TYPE15         (1 << 15)

/* Multiple function pin GPIOD control register */

#define GCR_GPD_MFP(n)           (1 << (n)) /* Bits 0-15: PDn pin function selection */
#  define GCR_GPD_MFP0           (1 << 0)
#  define GCR_GPD_MFP1           (1 << 1)
#  define GCR_GPD_MFP2           (1 << 2)
#  define GCR_GPD_MFP3           (1 << 3)
#  define GCR_GPD_MFP4           (1 << 4)
#  define GCR_GPD_MFP5           (1 << 5)
#  define GCR_GPD_MFP6           (1 << 6)
#  define GCR_GPD_MFP7           (1 << 7)
#  define GCR_GPD_MFP8           (1 << 8)
#  define GCR_GPD_MFP9           (1 << 9)
#  define GCR_GPD_MFP10          (1 << 10)
#  define GCR_GPD_MFP11          (1 << 11)
#  define GCR_GPD_MFP12          (1 << 12)
#  define GCR_GPD_MFP13          (1 << 13)
#  define GCR_GPD_MFP14          (1 << 14)
#  define GCR_GPD_MFP15          (1 << 15)
#define GCR_GPD_TYPE(n)          (1 << ((n)+16)) /* Bits 16-31: Enable Schmitt trigger function */
#  define GCR_GPD_TYPE0          (1 << 0)
#  define GCR_GPD_TYPE1          (1 << 1)
#  define GCR_GPD_TYPE2          (1 << 2)
#  define GCR_GPD_TYPE3          (1 << 3)
#  define GCR_GPD_TYPE4          (1 << 4)
#  define GCR_GPD_TYPE5          (1 << 5)
#  define GCR_GPD_TYPE6          (1 << 6)
#  define GCR_GPD_TYPE7          (1 << 7)
#  define GCR_GPD_TYPE8          (1 << 8)
#  define GCR_GPD_TYPE9          (1 << 9)
#  define GCR_GPD_TYPE10         (1 << 10)
#  define GCR_GPD_TYPE11         (1 << 11)
#  define GCR_GPD_TYPE12         (1 << 12)
#  define GCR_GPD_TYPE13         (1 << 13)
#  define GCR_GPD_TYPE14         (1 << 14)
#  define GCR_GPD_TYPE15         (1 << 15)

/* Multiple function pin GPIOE control register */

#define GCR_GPE_MFP(n)           (1 << (n)) /* Bits 0-15: PDn pin function selection */
#  define GCR_GPE_MFP0           (1 << 0)
#  define GCR_GPE_MFP1           (1 << 1)
#  define GCR_GPE_MFP2           (1 << 2)
#  define GCR_GPE_MFP3           (1 << 3)
#  define GCR_GPE_MFP4           (1 << 4)
#  define GCR_GPE_MFP5           (1 << 5)
#define GCR_GPE_TYPE(n)          (1 << ((n)+16)) /* Bits 16-31: Enable Schmitt trigger function */
#  define GCR_GPE_TYPE0          (1 << 0)
#  define GCR_GPE_TYPE1          (1 << 1)
#  define GCR_GPE_TYPE2          (1 << 2)
#  define GCR_GPE_TYPE3          (1 << 3)
#  define GCR_GPE_TYPE4          (1 << 4)
#  define GCR_GPE_TYPE5          (1 << 5)

/* Alternative multiple function pin control register */

#define GCR_ALT_MFP_PB10_S01     (1 << 0)  /* Bit 0:  Determines PB.10 function */
#define GCR_ALT_MFP_PB9_S11      (1 << 1)  /* Bit 1:  Determines PB.9 function */
#define GCR_ALT_MFP_PA7_S21      (1 << 2)  /* Bit 2:  Determines PA.7 function */
#define GCR_ALT_MFP_PB14_S31     (1 << 3)  /* Bit 3:  Determines PB.14 function */
#define GCR_ALT_MFP_PB11_PWM4    (1 << 4)  /* Bit 4:  Determines PB.11 function */
#define GCR_ALT_MFP_PC0_I2SRCLK  (1 << 5)  /* Bit 5:  Determines PC.0 function */
#define GCR_ALT_MFP_PC1_I2SBCLK  (1 << 6)  /* Bit 6:  Determines PC.1 function */
#define GCR_ALT_MFP_PC2_I2SD1    (1 << 7)  /* Bit 7:  Determines PC.2 function */
#define GCR_ALT_MFP_PC3_I2SD0    (1 << 8)  /* Bit 8:  Determines PC.3 function */
#define GCR_ALT_MFP_PA15_I2SMCLK (1 << 9)  /* Bit 9:  Determines PA.15 function */
#define GCR_ALT_MFP_PB12_CLKO    (1 << 10) /* Bit 10: Determines PB.12 function */
#define GCR_ALT_MFP_EBI_EN       (1 << 11) /* Bit 11: Determines PA.6, PA.7, PA.10, PA.11,
                                            * PB.6, PB.7, PB.12, PB.13, PC.6, PC.7, PC.14,
                                            * PC.15 function */
#define GCR_ALT_MFP_EBI_MCLK_EN  (1 << 12) /* Bit 12: Determines PC.8 function */
#define GCR_ALT_MFP_EBI_NWRL_EN  (1 << 13) /* Bit 13: Determines PB.2 function */
#define GCR_ALT_MFP_EBI_NWRH_WN  (1 << 14) /* Bit 14: Determines PB.3 function */
#define GCR_ALT_MFP_EBI_HB_EN0   (1 << 16) /* Bit 16: Determines PA.5 function */
#define GCR_ALT_MFP_EBI_HB_EN1   (1 << 17) /* Bit 17: Determines PA.4 function */
#define GCR_ALT_MFP_EBI_HB_EN2   (1 << 18) /* Bit 18: Determines PA.3 function */
#define GCR_ALT_MFP_EBI_HB_EN3   (1 << 19) /* Bit 19: Determines PA.2 function */
#define GCR_ALT_MFP_EBI_HB_EN4   (1 << 20) /* Bit 20: Determines PA.1 function */
#define GCR_ALT_MFP_EBI_HB_EN5   (1 << 21) /* Bit 21: Determines PA.12 function */
#define GCR_ALT_MFP_EBI_HB_EN6   (1 << 22) /* Bit 22: Determines PA.13 function */
#define GCR_ALT_MFP_EBI_HB_EN7   (1 << 23) /* Bit 23: Determines PA.14 function */

/* Register write-protection control register */

                                           /* Write: */
#define GCR_REGWRPROT_MASK       (0xff)    /* Bits 0-7: Register write protection code */
#  define GCR_REGWRPROT_1        (0x59)    /*           Disable sequence */
#  define GCR_REGWRPROT_2        (0x16)
#  define GCR_REGWRPROT_3        (0x88)
                                           /* Read: */
#define GCR_REGWRPROT_DIS        (1 << 0)  /* Bit 0: Register write protection disable index */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_NUC1XX_HARDWARE_NUC_GCR_H */
