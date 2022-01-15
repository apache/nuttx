/****************************************************************************
 * arch/arm/src/imx1/imx_wdog.h
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

#ifndef __ARCH_ARM_SRC_IMX1_IMX_WDOG_H
#define __ARCH_ARM_SRC_IMX1_IMX_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WDOG Register Offsets ****************************************************/

#define WDOG_WCR_OFFSET     0x0000 /* Watchdog Control Register */
#define WDOG_WSR_OFFSET     0x0004 /* Watchdog Service Register */
#define WDOG_WSTR_OFFSET    0x0008 /* Watchdog Status Register */

/* WDOG Register Addresses **************************************************/

#define IMX_WDOG_WCR        (IMX_WDOG_VBASE + WDOG_WCR_OFFSET)
#define IMX_WDOG_WSR        (IMX_WDOG_VBASE + WDOG_WSR_OFFSET)
#define IMX_WDOG_WSTRT      (IMX_WDOG_VBASE + WDOG_WSTR_OFFSET)

/* WDOG Register Bit Definitions ********************************************/

/* Watchdog Control Register */

#define WDOG_WCR_WDE        (1 << 0)  /* Bit 0: Watchdog Enable */
#define WDOG_WCR_WDEC       (1 << 1)  /* Bit 1: Watchdog Enable Control */
#define WDOG_WCR_SWR        (1 << 2)  /* Bit 2: Software Reset Enable */
#define WDOG_WCR_TMD        (1 << 3)  /* Bit 3: Test Mode Enable */
#define WDOG_WCR_WIE        (1 << 4)  /* Bit 4: Watchdog Interrupt Enable */

#define WDOG_WCR_WT_SHIFT   8 /* Bit 8-14: Watchdog Timeout */
#define WDOG_WCR_WT_MASK    (0x7f << WDOG_WCR_WT_SHIFT)
#define WDOG_WCR_WHALT      (1 << 15) /* Bit 15: Watchdog Halt */

/* Watchdog Service Register */

#define WDOG_WSR_SHIFT      0 /* Bit 0-15: Watchdog Service Register */
#define WDOG_WT_MASK        (0xffff << WDOG_WSR_SHIFT)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMX1_IMX_WDOG_H */
