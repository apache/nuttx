/****************************************************************************
 * arch/arm/include/eoss3/irq.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_EOSS3_IRQ_H
#define __ARCH_ARM_INCLUDE_EOSS3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/eoss3/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Common Processor Exceptions (vectors 0-15) */

#define EOSS3_IRQ_RESERVED       (0) /* Reserved vector */
                                     /* Vector  0: Reset stack pointer val */
                                     /* Vector  1: Reset (unused) */
#define EOSS3_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt */
#define EOSS3_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define EOSS3_IRQ_MEMFAULT       (4) /* Vector  4: Memory management */
#define EOSS3_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define EOSS3_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
                                     /* Vectors 7-10: Reserved */
#define EOSS3_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define EOSS3_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define EOSS3_IRQ_PENDSV        (14) /* Vector 14: Pendable sys srv req */
#define EOSS3_IRQ_SYSTICK       (15) /* Vector 15: System tick */
#define EOSS3_IRQ_EXTINT        (16) /* Vector num of first ext interrupt */

/* Chip-Specific External interrupts */

#define EOSS3_IRQ_SW2             (EOSS3_IRQ_EXTINT + 0)
#define EOSS3_IRQ_SW1             (EOSS3_IRQ_EXTINT + 1)
#define EOSS3_IRQ_RSV1            (EOSS3_IRQ_EXTINT + 2)
#define EOSS3_IRQ_FFE0MSG         (EOSS3_IRQ_EXTINT + 3)
#define EOSS3_IRQ_FBMSG           (EOSS3_IRQ_EXTINT + 4)
#define EOSS3_IRQ_GPIO            (EOSS3_IRQ_EXTINT + 5)
#define EOSS3_IRQ_SRAM_SLEEP      (EOSS3_IRQ_EXTINT + 6)
#define EOSS3_IRQ_UART            (EOSS3_IRQ_EXTINT + 7)
#define EOSS3_IRQ_TIMER           (EOSS3_IRQ_EXTINT + 8)
#define EOSS3_IRQ_CPUWD_INT       (EOSS3_IRQ_EXTINT + 9)
#define EOSS3_IRQ_CPUWD_RST       (EOSS3_IRQ_EXTINT + 0)
#define EOSS3_IRQ_BUS_TIMEOUT     (EOSS3_IRQ_EXTINT + 11)
#define EOSS3_IRQ_FPU             (EOSS3_IRQ_EXTINT + 12)
#define EOSS3_IRQ_PKFB            (EOSS3_IRQ_EXTINT + 13)
#define EOSS3_IRQ_RSV_I2S         (EOSS3_IRQ_EXTINT + 14)
#define EOSS3_IRQ_RSV_AUDIO       (EOSS3_IRQ_EXTINT + 15)
#define EOSS3_IRQ_SPI_MS          (EOSS3_IRQ_EXTINT + 16)
#define EOSS3_IRQ_CFG_DMA         (EOSS3_IRQ_EXTINT + 17)
#define EOSS3_IRQ_PMU_TIMER       (EOSS3_IRQ_EXTINT + 18)
#define EOSS3_IRQ_ADC_DONE        (EOSS3_IRQ_EXTINT + 19)
#define EOSS3_IRQ_RTC_ALARM       (EOSS3_IRQ_EXTINT + 20)
#define EOSS3_IRQ_RESET_INT       (EOSS3_IRQ_EXTINT + 21)
#define EOSS3_IRQ_FFE0            (EOSS3_IRQ_EXTINT + 22)
#define EOSS3_IRQ_FFE_WDT         (EOSS3_IRQ_EXTINT + 23)
#define EOSS3_IRQ_AP_BOOT         (EOSS3_IRQ_EXTINT + 24)
#define EOSS3_IRQ_LDO30_PG        (EOSS3_IRQ_EXTINT + 25)
#define EOSS3_IRQ_LDO50_PG        (EOSS3_IRQ_EXTINT + 26)
#define EOSS3_IRQ_SRAM_TO         (EOSS3_IRQ_EXTINT + 27)
#define EOSS3_IRQ_LPSD            (EOSS3_IRQ_EXTINT + 28)
#define EOSS3_IRQ_DMIC            (EOSS3_IRQ_EXTINT + 29)
#define EOSS3_IRQ_RSV2            (EOSS3_IRQ_EXTINT + 20)
#define EOSS3_IRQ_SDMA_DONE1      (EOSS3_IRQ_EXTINT + 31)
#define EOSS3_IRQ_SDMA_DONE2      (EOSS3_IRQ_EXTINT + 32)
#define EOSS3_IRQ_SDMA_DONE3      (EOSS3_IRQ_EXTINT + 33)
#define EOSS3_IRQ_SDMA_DONE4      (EOSS3_IRQ_EXTINT + 34)
#define EOSS3_IRQ_SDMA_DONE5      (EOSS3_IRQ_EXTINT + 35)
#define EOSS3_IRQ_SDMA_DONE6      (EOSS3_IRQ_EXTINT + 36)
#define EOSS3_IRQ_SDMA_DONE7      (EOSS3_IRQ_EXTINT + 37)
#define EOSS3_IRQ_SDMA_DONE8      (EOSS3_IRQ_EXTINT + 38)
#define EOSS3_IRQ_SDMA_DONE9      (EOSS3_IRQ_EXTINT + 39)
#define EOSS3_IRQ_SDMA_DONE10     (EOSS3_IRQ_EXTINT + 40)
#define EOSS3_IRQ_SDMA_DONE11     (EOSS3_IRQ_EXTINT + 41)
#define EOSS3_IRQ_AP_PDM_CLK_ON   (EOSS3_IRQ_EXTINT + 42)
#define EOSS3_IRQ_AP_PDM_CLK_OFF  (EOSS3_IRQ_EXTINT + 43)
#define EOSS3_IRQ_DMAC0_BLK_DONE  (EOSS3_IRQ_EXTINT + 44)
#define EOSS3_IRQ_DMAC0_BUF_DONE  (EOSS3_IRQ_EXTINT + 45)
#define EOSS3_IRQ_DMAC1_BLK_DONE  (EOSS3_IRQ_EXTINT + 46)
#define EOSS3_IRQ_DMAC1_BUF_DONE  (EOSS3_IRQ_EXTINT + 47)
#define EOSS3_IRQ_SDMA_DONE0      (EOSS3_IRQ_EXTINT + 48)
#define EOSS3_IRQ_SDMA_ERR        (EOSS3_IRQ_EXTINT + 49)
#define EOSS3_IRQ_I2S_SLV         (EOSS3_IRQ_EXTINT + 50)
#define EOSS3_IRQ_LPSD_VOICE_OFF  (EOSS3_IRQ_EXTINT + 51)
#define EOSS3_IRQ_DMIC_VOICE_OFF  (EOSS3_IRQ_EXTINT + 52)

#define EOSS3_NEXTINT             (53)
#define EOSS3_IRQ_NVECTORS        (EOSS3_IRQ_EXTINT + EOSS3_NEXTINT)
#define EOSS3_IRQ_INTERRUPTS      EOSS3_IRQ_EXTINT
#define NR_IRQS                   EOSS3_IRQ_NVECTORS

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* #define __ARCH_ARM_INCLUDE_EOSS3_IRQ_H */
