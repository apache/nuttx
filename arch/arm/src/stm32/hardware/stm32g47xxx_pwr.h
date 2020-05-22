/****************************************************************************
 *  arch/arm/src/stm32/hardware/stm32g47xxx_pwr.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_PWR_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_PWR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_PWR_CR1_OFFSET           0x0000        /* PWR Power Control Register 1 */
#define STM32_PWR_CR2_OFFSET           0x0004        /* PWR Power Control Register 2 */
#define STM32_PWR_CR3_OFFSET           0x0008        /* PWR Power Control Register 3 */
#define STM32_PWR_CR4_OFFSET           0x000c        /* PWR Power Control Register 4 */
#define STM32_PWR_SR1_OFFSET           0x0010        /* PWR Power Status Register 1 */
#define STM32_PWR_SR2_OFFSET           0x0014        /* PWR Power Status Register 2 */
#define STM32_PWR_SCR_OFFSET           0x0018        /* PWR Power Status Reset Register */
                                                     /* Offset 0x001c Reserved */
#define STM32_PWR_PUCRA_OFFSET         0x0020        /* Power Port A Pull Up Control Register */
#define STM32_PWR_PDCRA_OFFSET         0x0024        /* Power Port A Pull Down Control Register */
#define STM32_PWR_PUCRB_OFFSET         0x0028        /* Power Port B Pull Up Control Register */
#define STM32_PWR_PDCRB_OFFSET         0x002c        /* Power Port B Pull Down Control Register */
#define STM32_PWR_PUCRC_OFFSET         0x0030        /* Power Port C Pull Up Control Register */
#define STM32_PWR_PDCRC_OFFSET         0x0034        /* Power Port C Pull Down Control Register */
#define STM32_PWR_PUCRD_OFFSET         0x0038        /* Power Port D Pull Up Control Register */
#define STM32_PWR_PDCRD_OFFSET         0x003c        /* Power Port D Pull Down Control Register */
#define STM32_PWR_PUCRE_OFFSET         0x0040        /* Power Port E Pull Up Control Register */
#define STM32_PWR_PDCRE_OFFSET         0x0044        /* Power Port E Pull Down Control Register */
#define STM32_PWR_PUCRF_OFFSET         0x0048        /* Power Port F Pull Up Control Register */
#define STM32_PWR_PDCRF_OFFSET         0x004c        /* Power Port F Pull Down Control Register */
#define STM32_PWR_PUCRG_OFFSET         0x0050        /* Power Port G Pull Up Control Register */
#define STM32_PWR_PDCRG_OFFSET         0x0054        /* Power Port G Pull Down Control Register */
                                                     /* Offset 0x0058 Reserved */
                                                     /* Offset 0x005C Reserved */
                                                     /* Offset 0x0060 Reserved */
                                                     /* Offset 0x0064 Reserved */
                                                     /* Offset 0x0068 Reserved */
                                                     /* Offset 0x006C Reserved */
                                                     /* Offset 0x0070 Reserved */
                                                     /* Offset 0x0074 Reserved */
                                                     /* Offset 0x0078 Reserved */
                                                     /* Offset 0x007C Reserved */
#define STM32_PWR_CR5_OFFSET           0x0080        /* PWR Power Control Register 5 */

/* Register Addresses *******************************************************/

#define STM32_PWR_CR1                  (STM32_PWR_BASE + STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CR2                  (STM32_PWR_BASE + STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CR3                  (STM32_PWR_BASE + STM32_PWR_CR3_OFFSET)
#define STM32_PWR_CR4                  (STM32_PWR_BASE + STM32_PWR_CR4_OFFSET)
#define STM32_PWR_SR1                  (STM32_PWR_BASE + STM32_PWR_SR1_OFFSET)
#define STM32_PWR_SR2                  (STM32_PWR_BASE + STM32_PWR_SR2_OFFSET)
#define STM32_PWR_SCR                  (STM32_PWR_BASE + STM32_PWR_SCR_OFFSET)
#define STM32_PWR_PUCRA                (STM32_PWR_BASE + STM32_PWR_PUCRA_OFFSET)
#define STM32_PWR_PDCRA                (STM32_PWR_BASE + STM32_PWR_PDCRA_OFFSET)
#define STM32_PWR_PUCRB                (STM32_PWR_BASE + STM32_PWR_PUCRB_OFFSET)
#define STM32_PWR_PDCRB                (STM32_PWR_BASE + STM32_PWR_PDCRB_OFFSET)
#define STM32_PWR_PUCRC                (STM32_PWR_BASE + STM32_PWR_PUCRC_OFFSET)
#define STM32_PWR_PDCRC                (STM32_PWR_BASE + STM32_PWR_PDCRC_OFFSET)
#define STM32_PWR_PUCRD                (STM32_PWR_BASE + STM32_PWR_PUCRD_OFFSET)
#define STM32_PWR_PDCRD                (STM32_PWR_BASE + STM32_PWR_PDCRD_OFFSET)
#define STM32_PWR_PUCRE                (STM32_PWR_BASE + STM32_PWR_PUCRE_OFFSET)
#define STM32_PWR_PDCRE                (STM32_PWR_BASE + STM32_PWR_PDCRE_OFFSET)
#define STM32_PWR_PUCRF                (STM32_PWR_BASE + STM32_PWR_PUCRF_OFFSET)
#define STM32_PWR_PDCRF                (STM32_PWR_BASE + STM32_PWR_PDCRF_OFFSET)
#define STM32_PWR_PUCRG                (STM32_PWR_BASE + STM32_PWR_PUCRG_OFFSET)
#define STM32_PWR_PDCRG                (STM32_PWR_BASE + STM32_PWR_PDCRG_OFFSET)
#define STM32_PWR_CR5                  (STM32_PWR_BASE + STM32_PWR_CR5_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* PWR Power Control Register 1 (CR1) */

#define PWR_CR1_LPR_SHIFT              (14)                               /* Low Power Run */
#define PWR_CR1_LPR                    (0x1 << PWR_CR1_LPR_SHIFT)
#define PWR_CR1_VOS_SHIFT              (9)                                /* Voltage Scaling Range Selection */
#define PWR_CR1_VOS_MASK               (0x3 << PWR_CR1_VOS_SHIFT)
#  define PWR_CR1_VOS_RANGE_1          (0x1 << PWR_CR1_VOS_SHIFT)
#  define PWR_CR1_VOS_RANGE_2          (0x2 << PWR_CR1_VOS_SHIFT)
#define PWR_CR1_DBP_SHIFT              (8)                                /* Disable Backup Domain Write Protection */
#define PWR_CR1_DBP                    (0x1 << PWR_CR1_DBP_SHIFT)
#define PWR_CR1_LPMS_SHIFT             (0)                                /* Low Power Mode Selection */
#define PWR_CR1_LPMS_MASK              (0x7 << PWR_CR1_LPMS_SHIFT)
#  define PWR_CR1_LPMS_STOP_0          (0x0 << PWR_CR1_LPMS_SHIFT)        /* Stop 0 Mode */
#  define PWR_CR1_LPMS_STOP_1          (0x1 << PWR_CR1_LPMS_SHIFT)        /* Stop 1 Mode */
#  define PWR_CR1_LPMS_STANDBY         (0x3 << PWR_CR1_LPMS_SHIFT)        /* Standby Mode */
#  define PWR_CR1_LPMS_SHUTDOWN        (0x4 << PWR_CR1_LPMS_SHIFT)        /* Shutdown Mode */

/* PWR Power Control Register 2 (CR2) */

#define PWR_CR2_PVDE                   (1 << 0)                           /* Power Voltage Detector Enable */
#define PWR_CR2_PLS_SHIFT              (1)                                /* Power Voltage Detector Level Selection */
#define PWR_CR2_PLS_MASK               (0x7 << PWR_CR2_PLS_SHIFT)
#  define PWR_CR2_PLS_0                (0x0 << PWR_CR2_PLS_SHIFT)
#  define PWR_CR2_PLS_1                (0x1 << PWR_CR2_PLS_SHIFT)
#  define PWR_CR2_PLS_2                (0x2 << PWR_CR2_PLS_SHIFT)
#  define PWR_CR2_PLS_3                (0x3 << PWR_CR2_PLS_SHIFT)
#  define PWR_CR2_PLS_4                (0x4 << PWR_CR2_PLS_SHIFT)
#  define PWR_CR2_PLS_5                (0x5 << PWR_CR2_PLS_SHIFT)
#  define PWR_CR2_PLS_6                (0x6 << PWR_CR2_PLS_SHIFT)
#  define PWR_CR2_PLS_7                (0x7 << PWR_CR2_PLS_SHIFT)
#define PWR_CR2_PVMEN1                 (1 << 6)                           /* Peripheral Voltage Monitoring 3 Enable (VDDA vs ADC or COMP Minimum Voltage) */
#define PWR_CR2_PVMEN2                 (1 << 7)                           /* Peripheral Voltage Monitoring 4 Enable (VDDA vs DAC 1Msps or 15Msps Minimum Voltage) */

/* PWR Power Control Register 3 (CR3) */

#define PWR_CR3_EWUP1                  (1 << 0)                           /* Enable Wake Up Pin WKUP1 */
#define PWR_CR3_EWUP2                  (1 << 1)                           /* Enable Wake Up Pin WKUP2 */
#define PWR_CR3_EWUP3                  (1 << 2)                           /* Enable Wake Up Pin WKUP3 */
#define PWR_CR3_EWUP4                  (1 << 3)                           /* Enable Wake Up Pin WKUP4 */
#define PWR_CR3_EWUP5                  (1 << 4)                           /* Enable Wake Up Pin WKUP5 */
#define PWR_CR3_RRS                    (1 << 8)                           /* SRAM2 Retention In Standby Mode */
#define PWR_CR3_APC                    (1 << 10)                          /* Apply Pull Up And Pull Down Configuration */
#define PWR_CR3_UCPD1_STDBY            (1 << 13)                          /* USB Type C And Power Delivery Standby Mode */
#define PWR_CR3_UCPD1_DBDIS            (1 << 14)                          /* USB Type C And Power Delivery Dead Battery Disable */
#define PWR_CR3_EIWUL                  (1 << 15)                          /* Enable Internal Wake Up Line */

/* PWR Power Control Register 4 (CR4) */

#define PWR_CR4_WP1                    (1 << 0)                           /* Wake Up Pin WKUP1 polarity */
#define PWR_CR4_WP2                    (1 << 1)                           /* Wake Up Pin WKUP2 polarity */
#define PWR_CR4_WP3                    (1 << 2)                           /* Wake Up Pin WKUP3 polarity */
#define PWR_CR4_WP4                    (1 << 3)                           /* Wake Up Pin WKUP4 polarity */
#define PWR_CR4_WP5                    (1 << 4)                           /* Wake Up Pin WKUP5 polarity */
#define PWR_CR4_VBE                    (1 << 8)                           /* VBAT Battery charging Enable  */
#define PWR_CR4_VBRS                   (1 << 9)                           /* VBAT Battery charging Resistor Selection */

/* PWR Power Status Register 1 (SR1) */

#define PWR_SR1_WUF_SHIFT              (0)                                /* Wake Up Flags */
#define PWR_SR1_WUF_MASK               (0x1f << PWR_SR1_WUF_SHIFT)
#  define PWR_SR1_WUF1                 (0x1 << PWR_SR1_WUF_SHIFT)         /* Wake Up Flag 1 */
#  define PWR_SR1_WUF2                 (0x2 << PWR_SR1_WUF_SHIFT)         /* Wake Up Flag 2 */
#  define PWR_SR1_WUF3                 (0x4 << PWR_SR1_WUF_SHIFT)         /* Wake Up Flag 3 */
#  define PWR_SR1_WUF4                 (0x8 << PWR_SR1_WUF_SHIFT)         /* Wake Up Flag 4 */
#  define PWR_SR1_WUF5                 (0x10 << PWR_SR1_WUF_SHIFT)        /* Wake Up Flag 5 */
#define PWR_SR1_SBF                    (1 << 8)                           /* Stand-By Flag */
#define PWR_SR1_WUFI                   (1 << 15)                          /* Wake Up Flag Internal */

/* PWR Power Status Register 2 (SR2) */

#define PWR_SR2_FLASHRDY               (1 << 7)                           /* Flash Ready Flag */
#define PWR_SR2_REGLPS                 (1 << 8)                           /* Low-power Regulator Started */
#define PWR_SR2_REGLPF                 (1 << 9)                           /* Low-power Regulator Flag */
#define PWR_SR2_VOSF                   (1 << 10)                          /* Voltage Scaling Flag */
#define PWR_SR2_PVDO                   (1 << 11)                          /* Power Voltage Detector Output */
#define PWR_SR2_PVMO1                  (1 << 12)                          /* Peripheral Voltage Monitoring Output 1 */
#define PWR_SR2_PVMO2                  (1 << 13)                          /* Peripheral Voltage Monitoring Output 2 */
#define PWR_SR2_PVMO3                  (1 << 14)                          /* Peripheral Voltage Monitoring Output 3 */
#define PWR_SR2_PVMO4                  (1 << 15)                          /* Peripheral Voltage Monitoring Output 4 */

/* PWR Power Status Reset Register (SCR) */

#define PWR_SCR_CWUF_SHIFT             (0)                                /* Clear Wake Up Flags */
#define PWR_SCR_CWUF                   (0x1f << PWR_SCR_CWUF_SHIFT)
#  define PWR_SCR_CWUF1                (0x1 << PWR_SCR_CWUF_SHIFT)        /* Clear Wake Up Flag 1 */
#  define PWR_SCR_CWUF2                (0x2 << PWR_SCR_CWUF_SHIFT)        /* Clear Wake Up Flag 2 */
#  define PWR_SCR_CWUF3                (0x4 << PWR_SCR_CWUF_SHIFT)        /* Clear Wake Up Flag 3 */
#  define PWR_SCR_CWUF4                (0x8 << PWR_SCR_CWUF_SHIFT)        /* Clear Wake Up Flag 4 */
#  define PWR_SCR_CWUF5                (0x10 << PWR_SCR_CWUF_SHIFT)       /* Clear Wake Up Flag 5 */
#define PWR_SCR_CSBF                   (1 << 8)                           /* Clear Standby Flag */

/* Power Port A Pull Up Control Register (PUCRA) */

#define PWR_PUCRA_PA0                  (1 << 0)                           /* PA0 Pull Up Control Bit */
#define PWR_PUCRA_PA1                  (1 << 1)                           /* PA1 Pull Up Control Bit */
#define PWR_PUCRA_PA2                  (1 << 2)                           /* PA2 Pull Up Control Bit */
#define PWR_PUCRA_PA3                  (1 << 3)                           /* PA3 Pull Up Control Bit */
#define PWR_PUCRA_PA4                  (1 << 4)                           /* PA4 Pull Up Control Bit */
#define PWR_PUCRA_PA5                  (1 << 5)                           /* PA5 Pull Up Control Bit */
#define PWR_PUCRA_PA6                  (1 << 6)                           /* PA6 Pull Up Control Bit */
#define PWR_PUCRA_PA7                  (1 << 7)                           /* PA7 Pull Up Control Bit */
#define PWR_PUCRA_PA8                  (1 << 8)                           /* PA8 Pull Up Control Bit */
#define PWR_PUCRA_PA9                  (1 << 9)                           /* PA9 Pull Up Control Bit */
#define PWR_PUCRA_PA10                 (1 << 10)                          /* PA10 Pull Up Control Bit */
#define PWR_PUCRA_PA11                 (1 << 11)                          /* PA11 Pull Up Control Bit */
#define PWR_PUCRA_PA12                 (1 << 12)                          /* PA12 Pull Up Control Bit */
#define PWR_PUCRA_PA13                 (1 << 13)                          /* PA13 Pull Up Control Bit */
#define PWR_PUCRA_PA14                 (1 << 14)                          /* PA14 Pull Up Control Bit */
#define PWR_PUCRA_PA15                 (1 << 15)                          /* PA15 Pull Up Control Bit */

/* Power Port A Pull Down Control Register (PDCRA) */

#define PWR_PDCRA_PA0                  (1 << 0)                           /* PA0 Pull Down Control Bit */
#define PWR_PDCRA_PA1                  (1 << 1)                           /* PA1 Pull Down Control Bit */
#define PWR_PDCRA_PA2                  (1 << 2)                           /* PA2 Pull Down Control Bit */
#define PWR_PDCRA_PA3                  (1 << 3)                           /* PA3 Pull Down Control Bit */
#define PWR_PDCRA_PA4                  (1 << 4)                           /* PA4 Pull Down Control Bit */
#define PWR_PDCRA_PA5                  (1 << 5)                           /* PA5 Pull Down Control Bit */
#define PWR_PDCRA_PA6                  (1 << 6)                           /* PA6 Pull Down Control Bit */
#define PWR_PDCRA_PA7                  (1 << 7)                           /* PA7 Pull Down Control Bit */
#define PWR_PDCRA_PA8                  (1 << 8)                           /* PA8 Pull Down Control Bit */
#define PWR_PDCRA_PA9                  (1 << 9)                           /* PA9 Pull Down Control Bit */
#define PWR_PDCRA_PA10                 (1 << 10)                          /* PA10 Pull Down Control Bit */
#define PWR_PDCRA_PA11                 (1 << 11)                          /* PA11 Pull Down Control Bit */
#define PWR_PDCRA_PA12                 (1 << 12)                          /* PA12 Pull Down Control Bit */
#define PWR_PDCRA_PA13                 (1 << 13)                          /* PA13 Pull Down Control Bit */
#define PWR_PDCRA_PA14                 (1 << 14)                          /* PA14 Pull Down Control Bit */
#define PWR_PDCRA_PA15                 (1 << 15)                          /* PA15 Pull Down Control Bit */

/* Power Port B Pull Up Control Register (PUCRB) */

#define PWR_PUCRB_PB0                  (1 << 0)                           /* PB0 Pull Up Control Bit */
#define PWR_PUCRB_PB1                  (1 << 1)                           /* PB1 Pull Up Control Bit */
#define PWR_PUCRB_PB2                  (1 << 2)                           /* PB2 Pull Up Control Bit */
#define PWR_PUCRB_PB3                  (1 << 3)                           /* PB3 Pull Up Control Bit */
#define PWR_PUCRB_PB4                  (1 << 4)                           /* PB4 Pull Up Control Bit */
#define PWR_PUCRB_PB5                  (1 << 5)                           /* PB5 Pull Up Control Bit */
#define PWR_PUCRB_PB6                  (1 << 6)                           /* PB6 Pull Up Control Bit */
#define PWR_PUCRB_PB7                  (1 << 7)                           /* PB7 Pull Up Control Bit */
#define PWR_PUCRB_PB8                  (1 << 8)                           /* PB8 Pull Up Control Bit */
#define PWR_PUCRB_PB9                  (1 << 9)                           /* PB9 Pull Up Control Bit */
#define PWR_PUCRB_PB10                 (1 << 10)                          /* PB10 Pull Up Control Bit */
#define PWR_PUCRB_PB11                 (1 << 11)                          /* PB11 Pull Up Control Bit */
#define PWR_PUCRB_PB12                 (1 << 12)                          /* PB12 Pull Up Control Bit */
#define PWR_PUCRB_PB13                 (1 << 13)                          /* PB13 Pull Up Control Bit */
#define PWR_PUCRB_PB14                 (1 << 14)                          /* PB14 Pull Up Control Bit */
#define PWR_PUCRB_PB15                 (1 << 15)                          /* PB15 Pull Up Control Bit */

/* Power Port B Pull Down Control Register (PDCRB) */

#define PWR_PDCRB_PB0                  (1 << 0)                           /* PB0 Pull Down Control Bit */
#define PWR_PDCRB_PB1                  (1 << 1)                           /* PB1 Pull Down Control Bit */
#define PWR_PDCRB_PB2                  (1 << 2)                           /* PB2 Pull Down Control Bit */
#define PWR_PDCRB_PB3                  (1 << 3)                           /* PB3 Pull Down Control Bit */
#define PWR_PDCRB_PB4                  (1 << 4)                           /* PB4 Pull Down Control Bit */
#define PWR_PDCRB_PB5                  (1 << 5)                           /* PB5 Pull Down Control Bit */
#define PWR_PDCRB_PB6                  (1 << 6)                           /* PB6 Pull Down Control Bit */
#define PWR_PDCRB_PB7                  (1 << 7)                           /* PB7 Pull Down Control Bit */
#define PWR_PDCRB_PB8                  (1 << 8)                           /* PB8 Pull Down Control Bit */
#define PWR_PDCRB_PB9                  (1 << 9)                           /* PB9 Pull Down Control Bit */
#define PWR_PDCRB_PB10                 (1 << 10)                          /* PB10 Pull Down Control Bit */
#define PWR_PDCRB_PB11                 (1 << 11)                          /* PB11 Pull Down Control Bit */
#define PWR_PDCRB_PB12                 (1 << 12)                          /* PB12 Pull Down Control Bit */
#define PWR_PDCRB_PB13                 (1 << 13)                          /* PB13 Pull Down Control Bit */
#define PWR_PDCRB_PB14                 (1 << 14)                          /* PB14 Pull Down Control Bit */
#define PWR_PDCRB_PB15                 (1 << 15)                          /* PB15 Pull Down Control Bit */

/* Power Port C Pull Up Control Register (PUCRC) */

#define PWR_PUCRC_PC0                  (1 << 0)                           /* PC0 Pull Up Control Bit */
#define PWR_PUCRC_PC1                  (1 << 1)                           /* PC1 Pull Up Control Bit */
#define PWR_PUCRC_PC2                  (1 << 2)                           /* PC2 Pull Up Control Bit */
#define PWR_PUCRC_PC3                  (1 << 3)                           /* PC3 Pull Up Control Bit */
#define PWR_PUCRC_PC4                  (1 << 4)                           /* PC4 Pull Up Control Bit */
#define PWR_PUCRC_PC5                  (1 << 5)                           /* PC5 Pull Up Control Bit */
#define PWR_PUCRC_PC6                  (1 << 6)                           /* PC6 Pull Up Control Bit */
#define PWR_PUCRC_PC7                  (1 << 7)                           /* PC7 Pull Up Control Bit */
#define PWR_PUCRC_PC8                  (1 << 8)                           /* PC8 Pull Up Control Bit */
#define PWR_PUCRC_PC9                  (1 << 9)                           /* PC9 Pull Up Control Bit */
#define PWR_PUCRC_PC10                 (1 << 10)                          /* PC10 Pull Up Control Bit */
#define PWR_PUCRC_PC11                 (1 << 11)                          /* PC11 Pull Up Control Bit */
#define PWR_PUCRC_PC12                 (1 << 12)                          /* PC12 Pull Up Control Bit */
#define PWR_PUCRC_PC13                 (1 << 13)                          /* PC13 Pull Up Control Bit */
#define PWR_PUCRC_PC14                 (1 << 14)                          /* PC14 Pull Up Control Bit */
#define PWR_PUCRC_PC15                 (1 << 15)                          /* PC15 Pull Up Control Bit */

/* Power Port C Pull Down Control Register (PDCRC) */

#define PWR_PDCRC_PC0                  (1 << 0)                           /* PC0 Pull Down Control Bit */
#define PWR_PDCRC_PC1                  (1 << 1)                           /* PC1 Pull Down Control Bit */
#define PWR_PDCRC_PC2                  (1 << 2)                           /* PC2 Pull Down Control Bit */
#define PWR_PDCRC_PC3                  (1 << 3)                           /* PC3 Pull Down Control Bit */
#define PWR_PDCRC_PC4                  (1 << 4)                           /* PC4 Pull Down Control Bit */
#define PWR_PDCRC_PC5                  (1 << 5)                           /* PC5 Pull Down Control Bit */
#define PWR_PDCRC_PC6                  (1 << 6)                           /* PC6 Pull Down Control Bit */
#define PWR_PDCRC_PC7                  (1 << 7)                           /* PC7 Pull Down Control Bit */
#define PWR_PDCRC_PC8                  (1 << 8)                           /* PC8 Pull Down Control Bit */
#define PWR_PDCRC_PC9                  (1 << 9)                           /* PC9 Pull Down Control Bit */
#define PWR_PDCRC_PC10                 (1 << 10)                          /* PC10 Pull Down Control Bit */
#define PWR_PDCRC_PC11                 (1 << 11)                          /* PC11 Pull Down Control Bit */
#define PWR_PDCRC_PC12                 (1 << 12)                          /* PC12 Pull Down Control Bit */
#define PWR_PDCRC_PC13                 (1 << 13)                          /* PC13 Pull Down Control Bit */
#define PWR_PDCRC_PC14                 (1 << 14)                          /* PC14 Pull Down Control Bit */
#define PWR_PDCRC_PC15                 (1 << 15)                          /* PC15 Pull Down Control Bit */

/* Power Port D Pull Up Control Register (PUCRD) */

#define PWR_PUCRD_PD0                  (1 << 0)                           /* PD0 Pull Up Control Bit */
#define PWR_PUCRD_PD1                  (1 << 1)                           /* PD1 Pull Up Control Bit */
#define PWR_PUCRD_PD2                  (1 << 2)                           /* PD2 Pull Up Control Bit */
#define PWR_PUCRD_PD3                  (1 << 3)                           /* PD3 Pull Up Control Bit */
#define PWR_PUCRD_PD4                  (1 << 4)                           /* PD4 Pull Up Control Bit */
#define PWR_PUCRD_PD5                  (1 << 5)                           /* PD5 Pull Up Control Bit */
#define PWR_PUCRD_PD6                  (1 << 6)                           /* PD6 Pull Up Control Bit */
#define PWR_PUCRD_PD7                  (1 << 7)                           /* PD7 Pull Up Control Bit */
#define PWR_PUCRD_PD8                  (1 << 8)                           /* PD8 Pull Up Control Bit */
#define PWR_PUCRD_PD9                  (1 << 9)                           /* PD9 Pull Up Control Bit */
#define PWR_PUCRD_PD10                 (1 << 10)                          /* PD10 Pull Up Control Bit */
#define PWR_PUCRD_PD11                 (1 << 11)                          /* PD11 Pull Up Control Bit */
#define PWR_PUCRD_PD12                 (1 << 12)                          /* PD12 Pull Up Control Bit */
#define PWR_PUCRD_PD13                 (1 << 13)                          /* PD13 Pull Up Control Bit */
#define PWR_PUCRD_PD14                 (1 << 14)                          /* PD14 Pull Up Control Bit */
#define PWR_PUCRD_PD15                 (1 << 15)                          /* PD15 Pull Up Control Bit */

/* Power Port D Pull Down Control Register (PDCRD) */

#define PWR_PDCRD_PD0                  (1 << 0)                           /* PD0 Pull Down Control Bit */
#define PWR_PDCRD_PD1                  (1 << 1)                           /* PD1 Pull Down Control Bit */
#define PWR_PDCRD_PD2                  (1 << 2)                           /* PD2 Pull Down Control Bit */
#define PWR_PDCRD_PD3                  (1 << 3)                           /* PD3 Pull Down Control Bit */
#define PWR_PDCRD_PD4                  (1 << 4)                           /* PD4 Pull Down Control Bit */
#define PWR_PDCRD_PD5                  (1 << 5)                           /* PD5 Pull Down Control Bit */
#define PWR_PDCRD_PD6                  (1 << 6)                           /* PD6 Pull Down Control Bit */
#define PWR_PDCRD_PD7                  (1 << 7)                           /* PD7 Pull Down Control Bit */
#define PWR_PDCRD_PD8                  (1 << 8)                           /* PD8 Pull Down Control Bit */
#define PWR_PDCRD_PD9                  (1 << 9)                           /* PD9 Pull Down Control Bit */
#define PWR_PDCRD_PD10                 (1 << 10)                          /* PD10 Pull Down Control Bit */
#define PWR_PDCRD_PD11                 (1 << 11)                          /* PD11 Pull Down Control Bit */
#define PWR_PDCRD_PD12                 (1 << 12)                          /* PD12 Pull Down Control Bit */
#define PWR_PDCRD_PD13                 (1 << 13)                          /* PD13 Pull Down Control Bit */
#define PWR_PDCRD_PD14                 (1 << 14)                          /* PD14 Pull Down Control Bit */
#define PWR_PDCRD_PD15                 (1 << 15)                          /* PD15 Pull Down Control Bit */

/* Power Port E Pull Up Control Register (PUCRE) */

#define PWR_PUCRE_PE0                  (1 << 0)                           /* PE0 Pull Up Control Bit */
#define PWR_PUCRE_PE1                  (1 << 1)                           /* PE1 Pull Up Control Bit */
#define PWR_PUCRE_PE2                  (1 << 2)                           /* PE2 Pull Up Control Bit */
#define PWR_PUCRE_PE3                  (1 << 3)                           /* PE3 Pull Up Control Bit */
#define PWR_PUCRE_PE4                  (1 << 4)                           /* PE4 Pull Up Control Bit */
#define PWR_PUCRE_PE5                  (1 << 5)                           /* PE5 Pull Up Control Bit */
#define PWR_PUCRE_PE6                  (1 << 6)                           /* PE6 Pull Up Control Bit */
#define PWR_PUCRE_PE7                  (1 << 7)                           /* PE7 Pull Up Control Bit */
#define PWR_PUCRE_PE8                  (1 << 8)                           /* PE8 Pull Up Control Bit */
#define PWR_PUCRE_PE9                  (1 << 9)                           /* PE9 Pull Up Control Bit */
#define PWR_PUCRE_PE10                 (1 << 10)                          /* PE10 Pull Up Control Bit */
#define PWR_PUCRE_PE11                 (1 << 11)                          /* PE11 Pull Up Control Bit */
#define PWR_PUCRE_PE12                 (1 << 12)                          /* PE12 Pull Up Control Bit */
#define PWR_PUCRE_PE13                 (1 << 13)                          /* PE13 Pull Up Control Bit */
#define PWR_PUCRE_PE14                 (1 << 14)                          /* PE14 Pull Up Control Bit */
#define PWR_PUCRE_PE15                 (1 << 15)                          /* PE15 Pull Up Control Bit */

/* Power Port E Pull Down Control Register (PDCRE) */

#define PWR_PDCRE_PE0                  (1 << 0)                           /* PE0 Pull Down Control Bit */
#define PWR_PDCRE_PE1                  (1 << 1)                           /* PE1 Pull Down Control Bit */
#define PWR_PDCRE_PE2                  (1 << 2)                           /* PE2 Pull Down Control Bit */
#define PWR_PDCRE_PE3                  (1 << 3)                           /* PE3 Pull Down Control Bit */
#define PWR_PDCRE_PE4                  (1 << 4)                           /* PE4 Pull Down Control Bit */
#define PWR_PDCRE_PE5                  (1 << 5)                           /* PE5 Pull Down Control Bit */
#define PWR_PDCRE_PE6                  (1 << 6)                           /* PE6 Pull Down Control Bit */
#define PWR_PDCRE_PE7                  (1 << 7)                           /* PE7 Pull Down Control Bit */
#define PWR_PDCRE_PE8                  (1 << 8)                           /* PE8 Pull Down Control Bit */
#define PWR_PDCRE_PE9                  (1 << 9)                           /* PE9 Pull Down Control Bit */
#define PWR_PDCRE_PE10                 (1 << 10)                          /* PE10 Pull Down Control Bit */
#define PWR_PDCRE_PE11                 (1 << 11)                          /* PE11 Pull Down Control Bit */
#define PWR_PDCRE_PE12                 (1 << 12)                          /* PE12 Pull Down Control Bit */
#define PWR_PDCRE_PE13                 (1 << 13)                          /* PE13 Pull Down Control Bit */
#define PWR_PDCRE_PE14                 (1 << 14)                          /* PE14 Pull Down Control Bit */
#define PWR_PDCRE_PE15                 (1 << 15)                          /* PE15 Pull Down Control Bit */

/* Power Port F Pull Up Control Register (PUCRF) */

#define PWR_PUCRF_PF0                  (1 << 0)                           /* PF0 Pull Up Control Bit */
#define PWR_PUCRF_PF1                  (1 << 1)                           /* PF1 Pull Up Control Bit */
#define PWR_PUCRF_PF2                  (1 << 2)                           /* PF2 Pull Up Control Bit */
#define PWR_PUCRF_PF3                  (1 << 3)                           /* PF3 Pull Up Control Bit */
#define PWR_PUCRF_PF4                  (1 << 4)                           /* PF4 Pull Up Control Bit */
#define PWR_PUCRF_PF5                  (1 << 5)                           /* PF5 Pull Up Control Bit */
#define PWR_PUCRF_PF6                  (1 << 6)                           /* PF6 Pull Up Control Bit */
#define PWR_PUCRF_PF7                  (1 << 7)                           /* PF7 Pull Up Control Bit */
#define PWR_PUCRF_PF8                  (1 << 8)                           /* PF8 Pull Up Control Bit */
#define PWR_PUCRF_PF9                  (1 << 9)                           /* PF9 Pull Up Control Bit */
#define PWR_PUCRF_PF10                 (1 << 10)                          /* PF10 Pull Up Control Bit */
#define PWR_PUCRF_PF11                 (1 << 11)                          /* PF11 Pull Up Control Bit */
#define PWR_PUCRF_PF12                 (1 << 12)                          /* PF12 Pull Up Control Bit */
#define PWR_PUCRF_PF13                 (1 << 13)                          /* PF13 Pull Up Control Bit */
#define PWR_PUCRF_PF14                 (1 << 14)                          /* PF14 Pull Up Control Bit */
#define PWR_PUCRF_PF15                 (1 << 15)                          /* PF15 Pull Up Control Bit */

/* Power Port F Pull Down Control Register (PDCRF) */

#define PWR_PDCRF_PF0                  (1 << 0)                           /* PF0 Pull Down Control Bit */
#define PWR_PDCRF_PF1                  (1 << 1)                           /* PF1 Pull Down Control Bit */
#define PWR_PDCRF_PF2                  (1 << 2)                           /* PF2 Pull Down Control Bit */
#define PWR_PDCRF_PF3                  (1 << 3)                           /* PF3 Pull Down Control Bit */
#define PWR_PDCRF_PF4                  (1 << 4)                           /* PF4 Pull Down Control Bit */
#define PWR_PDCRF_PF5                  (1 << 5)                           /* PF5 Pull Down Control Bit */
#define PWR_PDCRF_PF6                  (1 << 6)                           /* PF6 Pull Down Control Bit */
#define PWR_PDCRF_PF7                  (1 << 7)                           /* PF7 Pull Down Control Bit */
#define PWR_PDCRF_PF8                  (1 << 8)                           /* PF8 Pull Down Control Bit */
#define PWR_PDCRF_PF9                  (1 << 9)                           /* PF9 Pull Down Control Bit */
#define PWR_PDCRF_PF10                 (1 << 10)                          /* PF10 Pull Down Control Bit */
#define PWR_PDCRF_PF11                 (1 << 11)                          /* PF11 Pull Down Control Bit */
#define PWR_PDCRF_PF12                 (1 << 12)                          /* PF12 Pull Down Control Bit */
#define PWR_PDCRF_PF13                 (1 << 13)                          /* PF13 Pull Down Control Bit */
#define PWR_PDCRF_PF14                 (1 << 14)                          /* PF14 Pull Down Control Bit */
#define PWR_PDCRF_PF15                 (1 << 15)                          /* PF15 Pull Down Control Bit */

/* Power Port G Pull Up Control Register (PUCRG) */

#define PWR_PUCRG_PG0                  (1 << 0)                           /* PG0 Pull Up Control Bit */
#define PWR_PUCRG_PG1                  (1 << 1)                           /* PG1 Pull Up Control Bit */
#define PWR_PUCRG_PG2                  (1 << 2)                           /* PG2 Pull Up Control Bit */
#define PWR_PUCRG_PG3                  (1 << 3)                           /* PG3 Pull Up Control Bit */
#define PWR_PUCRG_PG4                  (1 << 4)                           /* PG4 Pull Up Control Bit */
#define PWR_PUCRG_PG5                  (1 << 5)                           /* PG5 Pull Up Control Bit */
#define PWR_PUCRG_PG6                  (1 << 6)                           /* PG6 Pull Up Control Bit */
#define PWR_PUCRG_PG7                  (1 << 7)                           /* PG7 Pull Up Control Bit */
#define PWR_PUCRG_PG8                  (1 << 8)                           /* PG8 Pull Up Control Bit */
#define PWR_PUCRG_PG9                  (1 << 9)                           /* PG9 Pull Up Control Bit */
#define PWR_PUCRG_PG10                 (1 << 10)                          /* PG10 Pull Up Control Bit */
#define PWR_PUCRG_PG11                 (1 << 11)                          /* PG11 Pull Up Control Bit */
#define PWR_PUCRG_PG12                 (1 << 12)                          /* PG12 Pull Up Control Bit */
#define PWR_PUCRG_PG13                 (1 << 13)                          /* PG13 Pull Up Control Bit */
#define PWR_PUCRG_PG14                 (1 << 14)                          /* PG14 Pull Up Control Bit */
#define PWR_PUCRG_PG15                 (1 << 15)                          /* PG15 Pull Up Control Bit */

/* Power Port G Pull Down Control Register (PDCRG) */

#define PWR_PDCRG_PG0                  (1 << 0)                           /* PG0 Pull Down Control Bit */
#define PWR_PDCRG_PG1                  (1 << 1)                           /* PG1 Pull Down Control Bit */
#define PWR_PDCRG_PG2                  (1 << 2)                           /* PG2 Pull Down Control Bit */
#define PWR_PDCRG_PG3                  (1 << 3)                           /* PG3 Pull Down Control Bit */
#define PWR_PDCRG_PG4                  (1 << 4)                           /* PG4 Pull Down Control Bit */
#define PWR_PDCRG_PG5                  (1 << 5)                           /* PG5 Pull Down Control Bit */
#define PWR_PDCRG_PG6                  (1 << 6)                           /* PG6 Pull Down Control Bit */
#define PWR_PDCRG_PG7                  (1 << 7)                           /* PG7 Pull Down Control Bit */
#define PWR_PDCRG_PG8                  (1 << 8)                           /* PG8 Pull Down Control Bit */
#define PWR_PDCRG_PG9                  (1 << 9)                           /* PG9 Pull Down Control Bit */
#define PWR_PDCRG_PG10                 (1 << 10)                          /* PG10 Pull Down Control Bit */
#define PWR_PDCRG_PG11                 (1 << 11)                          /* PG11 Pull Down Control Bit */
#define PWR_PDCRG_PG12                 (1 << 12)                          /* PG12 Pull Down Control Bit */
#define PWR_PDCRG_PG13                 (1 << 13)                          /* PG13 Pull Down Control Bit */
#define PWR_PDCRG_PG14                 (1 << 14)                          /* PG14 Pull Down Control Bit */
#define PWR_PDCRG_PG15                 (1 << 15)                          /* PG15 Pull Down Control Bit */

/* PWR power control register 5 (CR5) */

#define PWR_CR5_R1MODE_SHIFT           (8)
#define PWR_CR5_R1MODE                 (0x1 << PWR_CR5_R1MODE_SHIFT)      /* Main Regulator Range 1 Mode */

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7X3XX_PWR_H */
