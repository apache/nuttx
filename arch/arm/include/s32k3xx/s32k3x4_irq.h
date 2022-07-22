/****************************************************************************
 * arch/arm/include/s32k3xx/s32k3x4_irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_S32K3XX_S32K3X4_IRQ_H
#define __ARCH_ARM_INCLUDE_S32K3XX_S32K3X4_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds to the vector number and hence maps directly to
 * bits in the NVIC.  This does, however, waste several words of memory in
 * the IRQ to handle mapping tables.
 */

/* External interrupts (vectors >= 16).
 * These definitions are chip-specific.
 */

/* CPU to CPU and Directed Interrupts */

#define S32K3XX_IRQ_CPU_TO_CPU1    (S32K3XX_IRQ_EXTINT +   0) /*   0: CPU to CPU interupt 0 (Core 0 --> Core 1) */
#define S32K3XX_IRQ_CPU_TO_CPU2    (S32K3XX_IRQ_EXTINT +   1) /*   1: CPU to CPU interupt 1 (Core 0 --> Core 1) */
#define S32K3XX_IRQ_CPU_TO_CPU3    (S32K3XX_IRQ_EXTINT +   2) /*   2: CPU to CPU interupt 2 (Core 0 <-- Core 1) */
#define S32K3XX_IRQ_CPU_TO_CPU4    (S32K3XX_IRQ_EXTINT +   3) /*   3: CPU to CPU interupt 3 (Core 0 <-- Core 1) */

/* Shared Peripheral Interrupts - On-Platform Vectors */

#define S32K3XX_IRQ_DMA_CH0        (S32K3XX_IRQ_EXTINT +   4) /*   4: DMA transfer complete and error CH0 */
#define S32K3XX_IRQ_DMA_CH1        (S32K3XX_IRQ_EXTINT +   5) /*   5: DMA transfer complete and error CH1 */
#define S32K3XX_IRQ_DMA_CH2        (S32K3XX_IRQ_EXTINT +   6) /*   6: DMA transfer complete and error CH2 */
#define S32K3XX_IRQ_DMA_CH3        (S32K3XX_IRQ_EXTINT +   7) /*   7: DMA transfer complete and error CH3 */
#define S32K3XX_IRQ_DMA_CH4        (S32K3XX_IRQ_EXTINT +   8) /*   8: DMA transfer complete and error CH4 */
#define S32K3XX_IRQ_DMA_CH5        (S32K3XX_IRQ_EXTINT +   9) /*   9: DMA transfer complete and error CH5 */
#define S32K3XX_IRQ_DMA_CH6        (S32K3XX_IRQ_EXTINT +  10) /*  10: DMA transfer complete and error CH6 */
#define S32K3XX_IRQ_DMA_CH7        (S32K3XX_IRQ_EXTINT +  11) /*  11: DMA transfer complete and error CH7 */
#define S32K3XX_IRQ_DMA_CH8        (S32K3XX_IRQ_EXTINT +  12) /*  12: DMA transfer complete and error CH8 */
#define S32K3XX_IRQ_DMA_CH9        (S32K3XX_IRQ_EXTINT +  13) /*  13: DMA transfer complete and error CH9 */
#define S32K3XX_IRQ_DMA_CH10       (S32K3XX_IRQ_EXTINT +  14) /*  14: DMA transfer complete and error CH10 */
#define S32K3XX_IRQ_DMA_CH11       (S32K3XX_IRQ_EXTINT +  15) /*  15: DMA transfer complete and error CH11 */
#define S32K3XX_IRQ_DMA_CH12       (S32K3XX_IRQ_EXTINT +  16) /*  16: DMA transfer complete and error CH12 */
#define S32K3XX_IRQ_DMA_CH13       (S32K3XX_IRQ_EXTINT +  17) /*  17: DMA transfer complete and error CH13 */
#define S32K3XX_IRQ_DMA_CH14       (S32K3XX_IRQ_EXTINT +  18) /*  18: DMA transfer complete and error CH14 */
#define S32K3XX_IRQ_DMA_CH15       (S32K3XX_IRQ_EXTINT +  19) /*  19: DMA transfer complete and error CH15 */
#define S32K3XX_IRQ_DMA_CH16       (S32K3XX_IRQ_EXTINT +  20) /*  20: DMA transfer complete and error CH16 */
#define S32K3XX_IRQ_DMA_CH17       (S32K3XX_IRQ_EXTINT +  21) /*  21: DMA transfer complete and error CH17 */
#define S32K3XX_IRQ_DMA_CH18       (S32K3XX_IRQ_EXTINT +  22) /*  22: DMA transfer complete and error CH18 */
#define S32K3XX_IRQ_DMA_CH19       (S32K3XX_IRQ_EXTINT +  23) /*  23: DMA transfer complete and error CH19 */
#define S32K3XX_IRQ_DMA_CH20       (S32K3XX_IRQ_EXTINT +  24) /*  24: DMA transfer complete and error CH20 */
#define S32K3XX_IRQ_DMA_CH21       (S32K3XX_IRQ_EXTINT +  25) /*  25: DMA transfer complete and error CH21 */
#define S32K3XX_IRQ_DMA_CH22       (S32K3XX_IRQ_EXTINT +  26) /*  26: DMA transfer complete and error CH22 */
#define S32K3XX_IRQ_DMA_CH23       (S32K3XX_IRQ_EXTINT +  27) /*  27: DMA transfer complete and error CH23 */
#define S32K3XX_IRQ_DMA_CH24       (S32K3XX_IRQ_EXTINT +  28) /*  28: DMA transfer complete and error CH24 */
#define S32K3XX_IRQ_DMA_CH25       (S32K3XX_IRQ_EXTINT +  29) /*  29: DMA transfer complete and error CH25 */
#define S32K3XX_IRQ_DMA_CH26       (S32K3XX_IRQ_EXTINT +  30) /*  30: DMA transfer complete and error CH26 */
#define S32K3XX_IRQ_DMA_CH27       (S32K3XX_IRQ_EXTINT +  31) /*  31: DMA transfer complete and error CH27 */
#define S32K3XX_IRQ_DMA_CH28       (S32K3XX_IRQ_EXTINT +  32) /*  32: DMA transfer complete and error CH28 */
#define S32K3XX_IRQ_DMA_CH29       (S32K3XX_IRQ_EXTINT +  33) /*  33: DMA transfer complete and error CH29 */
#define S32K3XX_IRQ_DMA_CH30       (S32K3XX_IRQ_EXTINT +  34) /*  34: DMA transfer complete and error CH30 */
#define S32K3XX_IRQ_DMA_CH31       (S32K3XX_IRQ_EXTINT +  35) /*  35: DMA transfer complete and error CH31 */
#define S32K3XX_IRQ_ERM_SBE        (S32K3XX_IRQ_EXTINT +  36) /*  36: Error Reporting Module single bit ECC error interrupt */
#define S32K3XX_IRQ_ERM_MBE        (S32K3XX_IRQ_EXTINT +  37) /*  37: Error Reporting Module multi bit ECC error interrupt */
#define S32K3XX_IRQ_MCM_FPU        (S32K3XX_IRQ_EXTINT +  38) /*  38: Miscellaneous Control Module FPU interrupts */
#define S32K3XX_IRQ_STM0           (S32K3XX_IRQ_EXTINT +  39) /*  39: System Timer Module 0 interrupt */
#define S32K3XX_IRQ_STM1           (S32K3XX_IRQ_EXTINT +  40) /*  40: System Timer Module 1 interrupt */
#define S32K3XX_IRQ_RESERVED41     (S32K3XX_IRQ_EXTINT +  41) /*  41: Reserved */
#define S32K3XX_IRQ_SWT0           (S32K3XX_IRQ_EXTINT +  42) /*  42: System Watchdog Timer 0 initial time-out interrupt */
#define S32K3XX_IRQ_SWT1           (S32K3XX_IRQ_EXTINT +  43) /*  43: System Watchdog Timer 1 initial time-out interrupt */
#define S32K3XX_IRQ_RESERVED44     (S32K3XX_IRQ_EXTINT +  44) /*  44: Reserved */
#define S32K3XX_IRQ_CTI0           (S32K3XX_IRQ_EXTINT +  45) /*  45: Cross Trigger Interface interrupt 0 */
#define S32K3XX_IRQ_CTI1           (S32K3XX_IRQ_EXTINT +  46) /*  46: Cross Trigger Interface interrupt 1 */
#define S32K3XX_IRQ_RESERVED47     (S32K3XX_IRQ_EXTINT +  47) /*  47: Reserved */

/* Shared Peripheral Interrupts - Off-Platform Vectors */

#define S32K3XX_IRQ_PFLASH_OP      (S32K3XX_IRQ_EXTINT +  48) /*  48: Platform Flash Memory Controller program or erase operation completed */
#define S32K3XX_IRQ_PFLASH_WDOG    (S32K3XX_IRQ_EXTINT +  49) /*  49: Platform Flash Memory Controller main/express watchdog time-out interrupt */
#define S32K3XX_IRQ_PFLASH_WDOGALT (S32K3XX_IRQ_EXTINT +  50) /*  50: Platform Flash Memory Controller alternate watchdog time-out interrupt */
#define S32K3XX_IRQ_MC_RGM         (S32K3XX_IRQ_EXTINT +  51) /*  51: Reset Generation Module interrupt */
#define S32K3XX_IRQ_PMC            (S32K3XX_IRQ_EXTINT +  52) /*  52: Power Management Controller interrupts */
#define S32K3XX_IRQ_SIUL2_VEC0     (S32K3XX_IRQ_EXTINT +  53) /*  53: System Integration Unit Lite 2 external interrupt vector 0 */
#define S32K3XX_IRQ_SIUL2_VEC1     (S32K3XX_IRQ_EXTINT +  54) /*  54: System Integration Unit Lite 2 external interrupt vector 1 */
#define S32K3XX_IRQ_SIUL2_VEC2     (S32K3XX_IRQ_EXTINT +  55) /*  55: System Integration Unit Lite 2 external interrupt vector 2 */
#define S32K3XX_IRQ_SIUL2_VEC3     (S32K3XX_IRQ_EXTINT +  56) /*  56: System Integration Unit Lite 2 external interrupt vector 3 */
#define S32K3XX_IRQ_RESERVED57     (S32K3XX_IRQ_EXTINT +  57) /*  57: Reserved */
#define S32K3XX_IRQ_RESERVED58     (S32K3XX_IRQ_EXTINT +  58) /*  58: Reserved */
#define S32K3XX_IRQ_RESERVED59     (S32K3XX_IRQ_EXTINT +  59) /*  59: Reserved */
#define S32K3XX_IRQ_RESERVED60     (S32K3XX_IRQ_EXTINT +  60) /*  60: Reserved */
#define S32K3XX_IRQ_EMIOS0_20_23   (S32K3XX_IRQ_EXTINT +  61) /*  61: eMIOS0 interrupt requests 20-23 */
#define S32K3XX_IRQ_EMIOS0_16_19   (S32K3XX_IRQ_EXTINT +  62) /*  62: eMIOS0 interrupt requests 16-19 */
#define S32K3XX_IRQ_EMIOS0_12_15   (S32K3XX_IRQ_EXTINT +  63) /*  63: eMIOS0 interrupt requests 12-15 */
#define S32K3XX_IRQ_EMIOS0_8_11    (S32K3XX_IRQ_EXTINT +  64) /*  64: eMIOS0 interrupt requests 8-11 */
#define S32K3XX_IRQ_EMIOS0_4_7     (S32K3XX_IRQ_EXTINT +  65) /*  65: eMIOS0 interrupt requests 4-7 */
#define S32K3XX_IRQ_EMIOS0_0_3     (S32K3XX_IRQ_EXTINT +  66) /*  66: eMIOS0 interrupt requests 0-3 */
#define S32K3XX_IRQ_RESERVED67     (S32K3XX_IRQ_EXTINT +  67) /*  67: Reserved */
#define S32K3XX_IRQ_RESERVED68     (S32K3XX_IRQ_EXTINT +  68) /*  68: Reserved */
#define S32K3XX_IRQ_EMIOS1_20_23   (S32K3XX_IRQ_EXTINT +  69) /*  69: eMIOS1 interrupt requests 20-23 */
#define S32K3XX_IRQ_EMIOS1_16_19   (S32K3XX_IRQ_EXTINT +  70) /*  70: eMIOS1 interrupt requests 16-19 */
#define S32K3XX_IRQ_EMIOS1_12_15   (S32K3XX_IRQ_EXTINT +  71) /*  71: eMIOS1 interrupt requests 12-15 */
#define S32K3XX_IRQ_EMIOS1_8_11    (S32K3XX_IRQ_EXTINT +  72) /*  72: eMIOS1 interrupt requests 8-11 */
#define S32K3XX_IRQ_EMIOS1_4_7     (S32K3XX_IRQ_EXTINT +  73) /*  73: eMIOS1 interrupt requests 4-7 */
#define S32K3XX_IRQ_EMIOS1_0_3     (S32K3XX_IRQ_EXTINT +  74) /*  74: eMIOS1 interrupt requests 0-3 */
#define S32K3XX_IRQ_RESERVED75     (S32K3XX_IRQ_EXTINT +  75) /*  75: Reserved */
#define S32K3XX_IRQ_RESERVED76     (S32K3XX_IRQ_EXTINT +  76) /*  76: Reserved */
#define S32K3XX_IRQ_EMIOS2_20_23   (S32K3XX_IRQ_EXTINT +  77) /*  77: eMIOS2 interrupt requests 20-23 */
#define S32K3XX_IRQ_EMIOS2_16_19   (S32K3XX_IRQ_EXTINT +  78) /*  78: eMIOS2 interrupt requests 16-19 */
#define S32K3XX_IRQ_EMIOS2_12_15   (S32K3XX_IRQ_EXTINT +  79) /*  79: eMIOS2 interrupt requests 12-15 */
#define S32K3XX_IRQ_EMIOS2_8_11    (S32K3XX_IRQ_EXTINT +  80) /*  80: eMIOS2 interrupt requests 8-11 */
#define S32K3XX_IRQ_EMIOS2_4_7     (S32K3XX_IRQ_EXTINT +  81) /*  81: eMIOS2 interrupt requests 4-7 */
#define S32K3XX_IRQ_EMIOS2_0_3     (S32K3XX_IRQ_EXTINT +  82) /*  82: eMIOS2 interrupt requests 0-3 */
#define S32K3XX_IRQ_WKPU           (S32K3XX_IRQ_EXTINT +  83) /*  83: Wakeup Unit interrupts */
#define S32K3XX_IRQ_CMU0           (S32K3XX_IRQ_EXTINT +  84) /*  84: Clock Monitoring Unit 0 interrupt */
#define S32K3XX_IRQ_CMU1           (S32K3XX_IRQ_EXTINT +  85) /*  85: Clock Monitoring Unit 1 interrupt */
#define S32K3XX_IRQ_CMU2           (S32K3XX_IRQ_EXTINT +  86) /*  86: Clock Monitoring Unit 2 interrupt */
#define S32K3XX_IRQ_BCTU           (S32K3XX_IRQ_EXTINT +  87) /*  87: Body Cross Triggering Unit interrupts */
#define S32K3XX_IRQ_RESERVED88     (S32K3XX_IRQ_EXTINT +  88) /*  88: Reserved */
#define S32K3XX_IRQ_RESERVED89     (S32K3XX_IRQ_EXTINT +  89) /*  89: Reserved */
#define S32K3XX_IRQ_RESERVED90     (S32K3XX_IRQ_EXTINT +  90) /*  90: Reserved */
#define S32K3XX_IRQ_RESERVED91     (S32K3XX_IRQ_EXTINT +  91) /*  91: Reserved */
#define S32k3XX_IRQ_LCU0           (S32K3XX_IRQ_EXTINT +  92) /*  92: Logic Control Unit 0 interrupts */
#define S32k3XX_IRQ_LCU1           (S32K3XX_IRQ_EXTINT +  93) /*  93: Logic Control Unit 1 interrupts */
#define S32K3XX_IRQ_RESERVED94     (S32K3XX_IRQ_EXTINT +  94) /*  94: Reserved */
#define S32K3XX_IRQ_RESERVED95     (S32K3XX_IRQ_EXTINT +  95) /*  95: Reserved */
#define S32k3XX_IRQ_PIT0           (S32K3XX_IRQ_EXTINT +  96) /*  96: Periodic Interrupt Timer 0 interrupts */
#define S32k3XX_IRQ_PIT1           (S32K3XX_IRQ_EXTINT +  97) /*  97: Periodic Interrupt Timer 1 interrupts */
#define S32k3XX_IRQ_PIT2           (S32K3XX_IRQ_EXTINT +  98) /*  98: Periodic Interrupt Timer 2 interrupts */
#define S32K3XX_IRQ_RESERVED99     (S32K3XX_IRQ_EXTINT +  99) /*  99: Reserved */
#define S32K3XX_IRQ_RESERVED100    (S32K3XX_IRQ_EXTINT + 100) /* 100: Reserved */
#define S32K3XX_IRQ_RESERVED101    (S32K3XX_IRQ_EXTINT + 101) /* 101: Reserved */
#define S32K3XX_IRQ_RTC            (S32K3XX_IRQ_EXTINT + 102) /* 102: Real Time Clock interrupts */
#define S32K3XX_IRQ_RESERVED103    (S32K3XX_IRQ_EXTINT + 103) /* 103: Reserved */
#define S32K3XX_IRQ_RESERVED104    (S32K3XX_IRQ_EXTINT + 104) /* 104: Reserved */
#define S32K3XX_IRQ_EMAC_COMMON    (S32K3XX_IRQ_EXTINT + 105) /* 105: Ethernet MAC common interrupts */
#define S32K3XX_IRQ_EMAC_TX        (S32K3XX_IRQ_EXTINT + 106) /* 106: Ethernet MAC TX interrupts */
#define S32K3XX_IRQ_EMAC_RX        (S32K3XX_IRQ_EXTINT + 107) /* 107: Ethernet MAC RX interrupts */
#define S32K3XX_IRQ_EMAC_SAFETY    (S32K3XX_IRQ_EXTINT + 108) /* 108: Ethernet MAC safety interrupts */
#define S32K3XX_IRQ_FLEXCAN0_0     (S32K3XX_IRQ_EXTINT + 109) /* 109: FlexCAN 0 interrupts 0 */
#define S32K3XX_IRQ_FLEXCAN0_1     (S32K3XX_IRQ_EXTINT + 110) /* 110: FlexCAN 0 interrupts 1 */
#define S32K3XX_IRQ_FLEXCAN0_2     (S32K3XX_IRQ_EXTINT + 111) /* 111: FlexCAN 0 interrupts 2 */
#define S32K3XX_IRQ_FLEXCAN0_3     (S32K3XX_IRQ_EXTINT + 112) /* 112: FlexCAN 0 interrupts 3 */
#define S32K3XX_IRQ_FLEXCAN1_0     (S32K3XX_IRQ_EXTINT + 113) /* 113: FlexCAN 1 interrupts 0 */
#define S32K3XX_IRQ_FLEXCAN1_1     (S32K3XX_IRQ_EXTINT + 114) /* 114: FlexCAN 1 interrupts 1 */
#define S32K3XX_IRQ_FLEXCAN1_2     (S32K3XX_IRQ_EXTINT + 115) /* 115: FlexCAN 1 interrupts 2 */
#define S32K3XX_IRQ_FLEXCAN2_0     (S32K3XX_IRQ_EXTINT + 116) /* 116: FlexCAN 2 interrupts 0 */
#define S32K3XX_IRQ_FLEXCAN2_1     (S32K3XX_IRQ_EXTINT + 117) /* 117: FlexCAN 2 interrupts 1 */
#define S32K3XX_IRQ_FLEXCAN2_2     (S32K3XX_IRQ_EXTINT + 118) /* 118: FlexCAN 2 interrupts 2 */
#define S32K3XX_IRQ_FLEXCAN3_0     (S32K3XX_IRQ_EXTINT + 119) /* 119: FlexCAN 3 interrupts 0 */
#define S32K3XX_IRQ_FLEXCAN3_1     (S32K3XX_IRQ_EXTINT + 120) /* 120: FlexCAN 3 interrupts 1 */
#define S32K3XX_IRQ_FLEXCAN4_0     (S32K3XX_IRQ_EXTINT + 121) /* 121: FlexCAN 4 interrupts 0 */
#define S32K3XX_IRQ_FLEXCAN4_1     (S32K3XX_IRQ_EXTINT + 122) /* 122: FlexCAN 4 interrupts 1 */
#define S32K3XX_IRQ_FLEXCAN5_0     (S32K3XX_IRQ_EXTINT + 123) /* 123: FlexCAN 5 interrupts 0 */
#define S32K3XX_IRQ_FLEXCAN5_1     (S32K3XX_IRQ_EXTINT + 124) /* 124: FlexCAN 5 interrupts 1 */
#define S32K3XX_IRQ_RESERVED125    (S32K3XX_IRQ_EXTINT + 125) /* 125: Reserved */
#define S32K3XX_IRQ_RESERVED126    (S32K3XX_IRQ_EXTINT + 126) /* 126: Reserved */
#define S32K3XX_IRQ_RESERVED127    (S32K3XX_IRQ_EXTINT + 127) /* 127: Reserved */
#define S32K3XX_IRQ_RESERVED128    (S32K3XX_IRQ_EXTINT + 128) /* 128: Reserved */
#define S32K3XX_IRQ_RESERVED129    (S32K3XX_IRQ_EXTINT + 129) /* 129: Reserved */
#define S32K3XX_IRQ_RESERVED130    (S32K3XX_IRQ_EXTINT + 130) /* 130: Reserved */
#define S32K3XX_IRQ_RESERVED131    (S32K3XX_IRQ_EXTINT + 131) /* 131: Reserved */
#define S32K3XX_IRQ_RESERVED132    (S32K3XX_IRQ_EXTINT + 132) /* 132: Reserved */
#define S32K3XX_IRQ_RESERVED133    (S32K3XX_IRQ_EXTINT + 133) /* 133: Reserved */
#define S32K3XX_IRQ_RESERVED134    (S32K3XX_IRQ_EXTINT + 134) /* 134: Reserved */
#define S32K3XX_IRQ_RESERVED135    (S32K3XX_IRQ_EXTINT + 135) /* 135: Reserved */
#define S32K3XX_IRQ_RESERVED136    (S32K3XX_IRQ_EXTINT + 136) /* 136: Reserved */
#define S32K3XX_IRQ_RESERVED137    (S32K3XX_IRQ_EXTINT + 137) /* 137: Reserved */
#define S32K3XX_IRQ_RESERVED138    (S32K3XX_IRQ_EXTINT + 138) /* 138: Reserved */
#define S32K3XX_IRQ_FLEXIO         (S32K3XX_IRQ_EXTINT + 139) /* 139: FlexIO interrupt */
#define S32K3XX_IRQ_RESERVED140    (S32K3XX_IRQ_EXTINT + 140) /* 140: Reserved */
#define S32K3XX_IRQ_LPUART0        (S32K3XX_IRQ_EXTINT + 141) /* 141: LPUART0 interrupts */
#define S32K3XX_IRQ_LPUART1        (S32K3XX_IRQ_EXTINT + 142) /* 142: LPUART1 interrupts */
#define S32K3XX_IRQ_LPUART2        (S32K3XX_IRQ_EXTINT + 143) /* 143: LPUART2 interrupts */
#define S32K3XX_IRQ_LPUART3        (S32K3XX_IRQ_EXTINT + 144) /* 144: LPUART3 interrupts */
#define S32K3XX_IRQ_LPUART4        (S32K3XX_IRQ_EXTINT + 145) /* 145: LPUART4 interrupts */
#define S32K3XX_IRQ_LPUART5        (S32K3XX_IRQ_EXTINT + 146) /* 146: LPUART5 interrupts */
#define S32K3XX_IRQ_LPUART6        (S32K3XX_IRQ_EXTINT + 147) /* 147: LPUART6 interrupts */
#define S32K3XX_IRQ_LPUART7        (S32K3XX_IRQ_EXTINT + 148) /* 148: LPUART7 interrupts */
#define S32K3XX_IRQ_LPUART8        (S32K3XX_IRQ_EXTINT + 149) /* 149: LPUART8 interrupts */
#define S32K3XX_IRQ_LPUART9        (S32K3XX_IRQ_EXTINT + 150) /* 150: LPUART9 interrupts */
#define S32K3XX_IRQ_LPUART10       (S32K3XX_IRQ_EXTINT + 151) /* 151: LPUART10 interrupts */
#define S32K3XX_IRQ_LPUART11       (S32K3XX_IRQ_EXTINT + 152) /* 152: LPUART11 interrupts */
#define S32K3XX_IRQ_LPUART12       (S32K3XX_IRQ_EXTINT + 153) /* 153: LPUART12 interrupts */
#define S32K3XX_IRQ_LPUART13       (S32K3XX_IRQ_EXTINT + 154) /* 154: LPUART13 interrupts */
#define S32K3XX_IRQ_LPUART14       (S32K3XX_IRQ_EXTINT + 155) /* 155: LPUART14 interrupts */
#define S32K3XX_IRQ_LPUART15       (S32K3XX_IRQ_EXTINT + 156) /* 156: LPUART15 interrupts */
#define S32K3XX_IRQ_RESERVED157    (S32K3XX_IRQ_EXTINT + 157) /* 157: Reserved */
#define S32K3XX_IRQ_RESERVED158    (S32K3XX_IRQ_EXTINT + 158) /* 158: Reserved */
#define S32K3XX_IRQ_RESERVED159    (S32K3XX_IRQ_EXTINT + 159) /* 159: Reserved */
#define S32K3XX_IRQ_RESERVED160    (S32K3XX_IRQ_EXTINT + 160) /* 160: Reserved */
#define S32K3XX_IRQ_LPI2C0         (S32K3XX_IRQ_EXTINT + 161) /* 161: LPI2C0 interrupts */
#define S32K3XX_IRQ_LPI2C1         (S32K3XX_IRQ_EXTINT + 162) /* 162: LPI2C1 interrupts */
#define S32K3XX_IRQ_RESERVED163    (S32K3XX_IRQ_EXTINT + 163) /* 163: Reserved */
#define S32K3XX_IRQ_RESERVED164    (S32K3XX_IRQ_EXTINT + 164) /* 164: Reserved */
#define S32K3XX_IRQ_LPSPI0         (S32K3XX_IRQ_EXTINT + 165) /* 165: LPSPI0 interrupt */
#define S32K3XX_IRQ_LPSPI1         (S32K3XX_IRQ_EXTINT + 166) /* 166: LPSPI1 interrupt */
#define S32K3XX_IRQ_LPSPI2         (S32K3XX_IRQ_EXTINT + 167) /* 167: LPSPI2 interrupt */
#define S32K3XX_IRQ_LPSPI3         (S32K3XX_IRQ_EXTINT + 168) /* 168: LPSPI3 interrupt */
#define S32K3XX_IRQ_LPSPI4         (S32K3XX_IRQ_EXTINT + 169) /* 169: LPSPI4 interrupt */
#define S32K3XX_IRQ_LPSPI5         (S32K3XX_IRQ_EXTINT + 170) /* 170: LPSPI5 interrupt */
#define S32K3XX_IRQ_RESERVED171    (S32K3XX_IRQ_EXTINT + 171) /* 171: Reserved */
#define S32K3XX_IRQ_RESERVED172    (S32K3XX_IRQ_EXTINT + 172) /* 172: Reserved */
#define S32K3XX_IRQ_QSPI           (S32K3XX_IRQ_EXTINT + 173) /* 173: Quad SPI interrupts */
#define S32K3XX_IRQ_SAI0           (S32K3XX_IRQ_EXTINT + 174) /* 174: SAI0 interrupts */
#define S32K3XX_IRQ_SAI1           (S32K3XX_IRQ_EXTINT + 175) /* 175: SAI1 interrupts */
#define S32K3XX_IRQ_RESERVED176    (S32K3XX_IRQ_EXTINT + 176) /* 176: Reserved */
#define S32K3XX_IRQ_RESERVED177    (S32K3XX_IRQ_EXTINT + 177) /* 177: Reserved */
#define S32K3XX_IRQ_JDC            (S32K3XX_IRQ_EXTINT + 178) /* 178: JTAG Data Communication interrupt */
#define S32K3XX_IRQ_RESERVED177    (S32K3XX_IRQ_EXTINT + 179) /* 179: Reserved */
#define S32K3XX_IRQ_ADC0           (S32K3XX_IRQ_EXTINT + 180) /* 180: ADC0 interrupts */
#define S32K3XX_IRQ_ADC1           (S32K3XX_IRQ_EXTINT + 181) /* 181: ADC1 interrupts */
#define S32K3XX_IRQ_ADC2           (S32K3XX_IRQ_EXTINT + 182) /* 182: ADC2 interrupts */
#define S32K3XX_IRQ_LPCMP0         (S32K3XX_IRQ_EXTINT + 183) /* 183: LPCMP0 interrupt */
#define S32K3XX_IRQ_LPCMP1         (S32K3XX_IRQ_EXTINT + 184) /* 184: LPCMP1 interrupt */
#define S32K3XX_IRQ_LPCMP2         (S32K3XX_IRQ_EXTINT + 185) /* 185: LPCMP2 interrupt */
#define S32K3XX_IRQ_RESERVED186    (S32K3XX_IRQ_EXTINT + 186) /* 186: Reserved */
#define S32K3XX_IRQ_RESERVED187    (S32K3XX_IRQ_EXTINT + 187) /* 187: Reserved */
#define S32K3XX_IRQ_RESERVED188    (S32K3XX_IRQ_EXTINT + 188) /* 188: Reserved */
#define S32K3XX_IRQ_FCCU0          (S32K3XX_IRQ_EXTINT + 189) /* 189: Fault Collection Control Unit interrupt 0 */
#define S32K3XX_IRQ_FCCU1          (S32K3XX_IRQ_EXTINT + 190) /* 190: Fault Collection Control Unit interrupt 1 */
#define S32K3XX_IRQ_STCU           (S32K3XX_IRQ_EXTINT + 191) /* 191: Self-Test Control Unit interrupts */
#define S32K3XX_IRQ_MU0_MUB_EX0    (S32K3XX_IRQ_EXTINT + 192) /* 192: Messaging Unit 0, Interface B, exception 0 */
#define S32K3XX_IRQ_MU0_MUB_EX1    (S32K3XX_IRQ_EXTINT + 193) /* 193: Messaging Unit 0, Interface B, exception 1 */
#define S32K3XX_IRQ_MU0_MUB_EX2    (S32K3XX_IRQ_EXTINT + 194) /* 194: Messaging Unit 0, Interface B, exception 2 */
#define S32K3XX_IRQ_MU1_MUB_EX0    (S32K3XX_IRQ_EXTINT + 195) /* 195: Messaging Unit 1, Interface B, exception 0 */
#define S32K3XX_IRQ_MU1_MUB_EX1    (S32K3XX_IRQ_EXTINT + 196) /* 196: Messaging Unit 1, Interface B, exception 1 */
#define S32K3XX_IRQ_MU1_MUB_EX2    (S32K3XX_IRQ_EXTINT + 197) /* 197: Messaging Unit 1, Interface B, exception 2 */
#define S32K3XX_IRQ_RESERVED198    (S32K3XX_IRQ_EXTINT + 198) /* 198: Reserved */
#define S32K3XX_IRQ_RESERVED199    (S32K3XX_IRQ_EXTINT + 199) /* 199: Reserved */
#define S32K3XX_IRQ_RESERVED200    (S32K3XX_IRQ_EXTINT + 200) /* 200: Reserved */
#define S32K3XX_IRQ_RESERVED201    (S32K3XX_IRQ_EXTINT + 201) /* 201: Reserved */
#define S32K3XX_IRQ_MU2_MUA_EX0    (S32K3XX_IRQ_EXTINT + 202) /* 202: Messaging Unit 2, Interface A, exception 0 */
#define S32K3XX_IRQ_MU2_MUA_EX1    (S32K3XX_IRQ_EXTINT + 203) /* 203: Messaging Unit 2, Interface A, exception 1 */
#define S32K3XX_IRQ_MU2_MUA_EX2    (S32K3XX_IRQ_EXTINT + 204) /* 204: Messaging Unit 2, Interface A, exception 2 */
#define S32K3XX_IRQ_MU2_MUB_EX0    (S32K3XX_IRQ_EXTINT + 205) /* 205: Messaging Unit 2, Interface B, exception 0 */
#define S32K3XX_IRQ_MU2_MUB_EX1    (S32K3XX_IRQ_EXTINT + 206) /* 206: Messaging Unit 2, Interface B, exception 1 */
#define S32K3XX_IRQ_MU2_MUB_EX2    (S32K3XX_IRQ_EXTINT + 207) /* 207: Messaging Unit 2, Interface B, exception 2 */
#define S32K3XX_IRQ_RST_FCCU       (S32K3XX_IRQ_EXTINT + 208) /* 208: FCCU failure to react interrupt */
#define S32K3XX_IRQ_RST_STCU       (S32K3XX_IRQ_EXTINT + 209) /* 209: STCU critical failure interrupt */
#define S32K3XX_IRQ_RST_RGM        (S32K3XX_IRQ_EXTINT + 210) /* 210: RGM functional reset escalation interrupt */
#define S32K3XX_IRQ_RST_CMU0       (S32K3XX_IRQ_EXTINT + 211) /* 211: CMU0 reset reaction interrupt */
#define S32K3XX_IRQ_RST_PLL_LOL    (S32K3XX_IRQ_EXTINT + 212) /* 212: PLL Loss of Lock (LOL) interrupt */
#define S32K3XX_IRQ_RST_CORECLK    (S32K3XX_IRQ_EXTINT + 213) /* 213: CORE_CLK_FAIL CMU reset reaction interrupt */
#define S32K3XX_IRQ_RESERVED214    (S32K3XX_IRQ_EXTINT + 214) /* 214: Reserved */
#define S32K3XX_IRQ_RST_AIPSPCLK   (S32K3XX_IRQ_EXTINT + 215) /* 215: AIPS_PLAT_CLK_FAIL CMU reset reaction interrupt */
#define S32K3XX_IRQ_RESERVED216    (S32K3XX_IRQ_EXTINT + 216) /* 216: Reserved */
#define S32K3XX_IRQ_RST_HSECLK     (S32K3XX_IRQ_EXTINT + 217) /* 217: HSE_CLK_FAIL CMU reset reaction interrupt */
#define S32K3XX_IRQ_RST_CGMCLK     (S32K3XX_IRQ_EXTINT + 218) /* 218: Clock Generation Module clkdiv failed */
#define S32K3XX_IRQ_RESERVED219    (S32K3XX_IRQ_EXTINT + 219) /* 219: Reserved */
#define S32K3XX_IRQ_RST_HSE        (S32K3XX_IRQ_EXTINT + 220) /* 220: HSE Tamper interrupt */
#define S32K3XX_IRQ_RST_HSESNVS    (S32K3XX_IRQ_EXTINT + 221) /* 221: HSE SNVS Tamper interrupt */
#define S32K3XX_IRQ_RST_MDMDAP     (S32K3XX_IRQ_EXTINT + 222) /* 222: MDM DAP destructive reset interrupt */
#define S32K3XX_IRQ_RST_PIN        (S32K3XX_IRQ_EXTINT + 223) /* 223: Pin reset interrupt */
#define S32K3XX_IRQ_RESERVED224    (S32K3XX_IRQ_EXTINT + 224) /* 224: Reserved */
#define S32K3XX_IRQ_RESERVED225    (S32K3XX_IRQ_EXTINT + 225) /* 225: Reserved */
#define S32K3XX_IRQ_RESERVED226    (S32K3XX_IRQ_EXTINT + 226) /* 226: Reserved */
#define S32K3XX_IRQ_RESERVED227    (S32K3XX_IRQ_EXTINT + 227) /* 227: Reserved */
#define S32K3XX_IRQ_RESERVED228    (S32K3XX_IRQ_EXTINT + 228) /* 228: Reserved */
#define S32K3XX_IRQ_RESERVED229    (S32K3XX_IRQ_EXTINT + 229) /* 229: Reserved */
#define S32K3XX_IRQ_RESERVED230    (S32K3XX_IRQ_EXTINT + 230) /* 230: Reserved */
#define S32K3XX_IRQ_RESERVED231    (S32K3XX_IRQ_EXTINT + 231) /* 231: Reserved */

#define S32K3XX_IRQ_NEXTINT        (232)
#define S32K3XX_IRQ_NIRQS          (S32K3XX_IRQ_EXTINT + S32K3XX_IRQ_NEXTINT)

/* Total number of IRQs */

#define NR_IRQS                    (S32K3XX_IRQ_NIRQS)

#endif /* __ARCH_ARM_INCLUDE_S32K3XX_S32K3X4_IRQ_H */
