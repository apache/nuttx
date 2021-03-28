/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_evntrtr.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EVNTRTR_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EVNTRTR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define LPC43_EVNTRTR_HILO_OFFSET    0x0000 /* Level configuration register */
#define LPC43_EVNTRTR_EDGE_OFFSET    0x0004 /* Edge configuration */

#define LPC43_EVNTRTR_CLREN_OFFSET   0x0fd8 /* Clear event enable register */
#define LPC43_EVNTRTR_SETEN_OFFSET   0x0fdc /* Set event enable register */
#define LPC43_EVNTRTR_STATUS_OFFSET  0x0fe0 /* Event Status register */
#define LPC43_EVNTRTR_ENABLE_OFFSET  0x0fe4 /* Event Enable register */
#define LPC43_EVNTRTR_CLRSTAT_OFFSET 0x0fe8 /* Clear event status register */
#define LPC43_EVNTRTR_SETSTAT_OFFSET 0x0fec /* Set event status register */

/* Register Addresses *******************************************************/

#define LPC43_EVNTRTR_HILO           (LPC43_EVNTRTR_BASE+LPC43_EVNTRTR_HILO_OFFSET)
#define LPC43_EVNTRTR_EDGE           (LPC43_EVNTRTR_BASE+LPC43_EVNTRTR_EDGE_OFFSET)

#define LPC43_EVNTRTR_CLREN          (LPC43_EVNTRTR_BASE+LPC43_EVNTRTR_CLREN_OFFSET)
#define LPC43_EVNTRTR_SETEN          (LPC43_EVNTRTR_BASE+LPC43_EVNTRTR_SETEN_OFFSET)
#define LPC43_EVNTRTR_STATUS         (LPC43_EVNTRTR_BASE+LPC43_EVNTRTR_STATUS_OFFSET)
#define LPC43_EVNTRTR_ENABLE         (LPC43_EVNTRTR_BASE+LPC43_EVNTRTR_ENABLE_OFFSET)
#define LPC43_EVNTRTR_CLRSTAT        (LPC43_EVNTRTR_BASE+LPC43_EVNTRTR_CLRSTAT_OFFSET)
#define LPC43_EVNTRTR_SETSTAT        (LPC43_EVNTRTR_BASE+LPC43_EVNTRTR_SETSTAT_OFFSET)

/* Register Bit Definitions *************************************************/

/* Event router inputs.  Bit settings common to all registers */

#define EVNTRTR_SOURCE_WAKEUP0       0  /* WAKEUP0 pin */
#define EVNTRTR_SOURCE_WAKEUP1       1  /* WAKEUP1 pin */
#define EVNTRTR_SOURCE_WAKEUP2       2  /* WAKEUP2 pin */
#define EVNTRTR_SOURCE_WAKEUP3       3  /* WAKEUP3 pin */
#define EVNTRTR_SOURCE_ATIMER        4  /* Alarm timer interrupt */
#define EVNTRTR_SOURCE_RTC           5  /* RTC interrupt and event recorder/monitor interrupt */
#define EVNTRTR_SOURCE_BOD           6  /* BOD trip level 1interrupt */
#define EVNTRTR_SOURCE_WWDT          7  /* WWDT interrupt */
#define EVNTRTR_SOURCE_ETHERNET      8  /* Ethernet wake-up packet indicator */
#define EVNTRTR_SOURCE_USB0          9  /* USB0 wake-up request signal */
#define EVNTRTR_SOURCE_USB1          10 /* USB1 AHB_NEED_CLK signal */
#define EVNTRTR_SOURCE_SDMMC         11 /* SD/MMC interrupt */
#define EVNTRTR_SOURCE_CAN           12 /* C_CAN0 | C_CAN1 interrupt */
#define EVNTRTR_SOURCE_TIM2          13 /* Combined timer output 2 (SCT output 2 | TIMER0 Ch2) */
#define EVNTRTR_SOURCE_TIM6          14 /* Combined timer output 6 (SCT output 6 | TIMER1 Ch2) */
#define EVNTRTR_SOURCE_QEI           15 /* QEI interrupt */
#define EVNTRTR_SOURCE_TIM14         16 /* Combined timer output 14 (SCT output 14 | TIMER3 Ch2) */
                                        /* 17-18: Reserved */
#define EVNTRTR_SOURCE_RESET         19 /* Reset event */

#define EVNTRTR_WAKEUP0              (1 << EVNTRTR_SOURCE_WAKEUP0)
#define EVNTRTR_WAKEUP1              (1 << EVNTRTR_SOURCE_WAKEUP1)
#define EVNTRTR_WAKEUP2              (1 << EVNTRTR_SOURCE_WAKEUP2)
#define EVNTRTR_WAKEUP3              (1 << EVNTRTR_SOURCE_WAKEUP3)
#define EVNTRTR_ATIMER               (1 << EVNTRTR_SOURCE_ATIMER)
#define EVNTRTR_RTC                  (1 << EVNTRTR_SOURCE_RTC)
#define EVNTRTR_BOD                  (1 << EVNTRTR_SOURCE_BOD)
#define EVNTRTR_WWDT                 (1 << EVNTRTR_SOURCE_WWDT)
#define EVNTRTR_ETH                  (1 << EVNTRTR_SOURCE_ETHERNET)
#define EVNTRTR_USB0                 (1 << EVNTRTR_SOURCE_USB0)
#define EVNTRTR_USB1                 (1 << EVNTRTR_SOURCE_USB1)
#define EVNTRTR_SDMMC                (1 << EVNTRTR_SOURCE_SDMMC)
#define EVNTRTR_CAN                  (1 << EVNTRTR_SOURCE_CAN)
#define EVNTRTR_TIM2                 (1 << EVNTRTR_SOURCE_TIM2)
#define EVNTRTR_TIM6                 (1 << EVNTRTR_SOURCE_TIM6)
#define EVNTRTR_QEI                  (1 << EVNTRTR_SOURCE_QEI)
#define EVNTRTR_TIM14                (1 << EVNTRTR_SOURCE_TIM14)
#define EVNTRTR_RESET                (1 << EVNTRTR_SOURCE_RESET)

/* Level configuration register */

#define EVNTRTR_HILO(n)              (1 << (n))

/* Edge configuration */

#define EVNTRTR_EDGE(n)              (1 << (n))

/* Clear event enable register */

#define EVNTRTR_CLREN(n)             (1 << (n))

/* Set event enable register */

#define EVNTRTR_SETEN(n)             (1 << (n))

/* Event Status register */

#define EVNTRTR_STATUS(n)            (1 << (n))

/* Event Enable register */

#define EVNTRTR_ENABLE(n)            (1 << (n))

/* Clear event status register */

#define EVNTRTR_CLRSTAT(n)           (1 << (n))

/* Set event status register */

#define EVNTRTR_SETSTAT(n)           (1 << (n))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_EVNTRTR_H */
