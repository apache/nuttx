/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_wdt.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_WDT_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/cxd56xx/chip.h>

#include "hardware/cxd5602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WDT register addresses ***************************************************/

#define CXD56_WDT_WDOGLOAD      (CXD56_WDOG_BASE + 0x0000) /* Load register */
#define CXD56_WDT_WDOGVALUE     (CXD56_WDOG_BASE + 0x0004) /* Value register [RO] */
#define CXD56_WDT_WDOGCONTROL   (CXD56_WDOG_BASE + 0x0008) /* Control register */
#define CXD56_WDT_WDOGINTCLR    (CXD56_WDOG_BASE + 0x000C) /* Clear Interrupt register [WO] */
#define CXD56_WDT_WDOGRIS       (CXD56_WDOG_BASE + 0x0010) /* Raw Interrupt Status register [RO] */
#define CXD56_WDT_WDOGMIS       (CXD56_WDOG_BASE + 0x0014) /* Interrupt Status register [RO] */
#define CXD56_WDT_WDOGLOCK      (CXD56_WDOG_BASE + 0x0C00) /* Lock register */
#define CXD56_WDT_WDOGITCR      (CXD56_WDOG_BASE + 0x0F00) /* Integration Test Control register */
#define CXD56_WDT_WDOGITOP      (CXD56_WDOG_BASE + 0x0F04) /* Integration Test Output register [WO] */
#define CXD56_WDT_WDOGPERIPHID0 (CXD56_WDOG_BASE + 0x0FE0) /* Peripheral ID0 register [RO] */
#define CXD56_WDT_WDOGPERIPHID1 (CXD56_WDOG_BASE + 0x0FE4) /* Peripheral ID1 register [RO] */
#define CXD56_WDT_WDOGPERIPHID2 (CXD56_WDOG_BASE + 0x0FE8) /* Peripheral ID2 register [RO] */
#define CXD56_WDT_WDOGPERIPHID3 (CXD56_WDOG_BASE + 0x0FFC) /* Peripheral ID3 register [RO] */
#define CXD56_WDT_WDOGPCELLID0  (CXD56_WDOG_BASE + 0x0FF0) /* PrimeCell ID0 register [RO] */
#define CXD56_WDT_WDOGPCELLID1  (CXD56_WDOG_BASE + 0x0FF4) /* PrimeCell ID1 register [RO] */
#define CXD56_WDT_WDOGPCELLID2  (CXD56_WDOG_BASE + 0x0FF8) /* PrimeCell ID2 register [RO] */
#define CXD56_WDT_WDOGPCELLID3  (CXD56_WDOG_BASE + 0x0FFC) /* PrimeCell ID3 register [RO] */

/* WDT register bit definitions *********************************************/

/* Control Register */

#define WDOGCONTROL_RESEN       (0x1 << 1)  /* enable reset output */
#define WDOGCONTROL_INTEN       (0x1 << 0)  /* enable interrupt output */
#define WDOGCONTROL_STOP        (0x0)       /* stop */

/* Interrupt Register */

#define WDOGRIS_RAWINT          (0x1 << 0)  /* raw interrupt status */
#define WDOGRIS_INT             (0x1 << 0)  /* interrupt status */

/* Lock Register */

#define WDOGLOCK_UNLOCK_KEY     (0x1ACCE551) /* unlock key */
#define WDOGLOCK_ACCESS_ENABLE  (0x0 << 0)   /* enable write access */
#define WDOGLOCK_ACCESS_DISABLE (0x1 << 0)   /* disable write access */

/* Test Register  */

#define WDOGITCR_ENABLE         (0x1 << 0)   /* enable test mode */
#define WDOGITOP_WDOGINT        (0x1 << 1)   /* output interrupt */
#define WDOGITOP_WDOGRES        (0x1 << 0)   /* output reset */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_WDT_H */
