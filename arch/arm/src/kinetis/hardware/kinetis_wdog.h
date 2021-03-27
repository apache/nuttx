/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_wdog.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_WDOG_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_WDOG_STCTRLH_OFFSET  0x0000 /* Watchdog Status and Control Register High */
#define KINETIS_WDOG_STCTRLL_OFFSET  0x0002 /* Watchdog Status and Control Register Low */
#define KINETIS_WDOG_TOVALH_OFFSET   0x0004 /* Watchdog Time-out Value Register High */
#define KINETIS_WDOG_TOVALL_OFFSET   0x0006 /* Watchdog Time-out Value Register Low */
#define KINETIS_WDOG_WINH_OFFSET     0x0008 /* Watchdog Window Register High */
#define KINETIS_WDOG_WINL_OFFSET     0x000a /* Watchdog Window Register Low */
#define KINETIS_WDOG_REFRESH_OFFSET  0x000c /* Watchdog Refresh Register */
#define KINETIS_WDOG_UNLOCK_OFFSET   0x000e /* Watchdog Unlock Register */
#define KINETIS_WDOG_TMROUTH_OFFSET  0x0010 /* Watchdog Timer Output Register High */
#define KINETIS_WDOG_TMROUTL_OFFSET  0x0012 /* Watchdog Timer Output Register Low */
#define KINETIS_WDOG_RSTCNT_OFFSET   0x0014 /* Watchdog Reset Count Register */
#define KINETIS_WDOG_PRESC_OFFSET    0x0016 /* Watchdog Prescaler Register */

/* Register Addresses *******************************************************/

#define KINETIS_WDOG_STCTRLH         (KINETIS_WDOG_BASE+KINETIS_WDOG_STCTRLH_OFFSET)
#define KINETIS_WDOG_STCTRLL         (KINETIS_WDOG_BASE+KINETIS_WDOG_STCTRLL_OFFSET)
#define KINETIS_WDOG_TOVALH          (KINETIS_WDOG_BASE+KINETIS_WDOG_TOVALH_OFFSET)
#define KINETIS_WDOG_TOVALL          (KINETIS_WDOG_BASE+KINETIS_WDOG_TOVALL_OFFSET)
#define KINETIS_WDOG_WINH            (KINETIS_WDOG_BASE+KINETIS_WDOG_WINH_OFFSET)
#define KINETIS_WDOG_WINL            (KINETIS_WDOG_BASE+KINETIS_WDOG_WINL_OFFSET)
#define KINETIS_WDOG_REFRESH         (KINETIS_WDOG_BASE+KINETIS_WDOG_REFRESH_OFFSET)
#define KINETIS_WDOG_UNLOCK          (KINETIS_WDOG_BASE+KINETIS_WDOG_UNLOCK_OFFSET)
#define KINETIS_WDOG_TMROUTH         (KINETIS_WDOG_BASE+KINETIS_WDOG_TMROUTH_OFFSET)
#define KINETIS_WDOG_TMROUTL         (KINETIS_WDOG_BASE+KINETIS_WDOG_TMROUTL_OFFSET)
#define KINETIS_WDOG_RSTCNT          (KINETIS_WDOG_BASE+KINETIS_WDOG_RSTCNT_OFFSET)
#define KINETIS_WDOG_PRESC           (KINETIS_WDOG_BASE+KINETIS_WDOG_PRESC_OFFSET)

/* Register Bit Definitions *************************************************/

/* Watchdog Status and Control Register High (16-bit) */

#define WDOG_STCTRLH_WDOGEN          (1 << 0)  /* Bit 0:  Enables or disables the WDOG�s operation */
#define WDOG_STCTRLH_CLKSRC          (1 << 1)  /* Bit 1:  Selects clock source for the WDOG timer */
#define WDOG_STCTRLH_IRQRSTEN        (1 << 2)  /* Bit 2:  Enable the debug breadcrumbs feature */
#define WDOG_STCTRLH_WINEN           (1 << 3)  /* Bit 3:  Enable windowing mode */
#define WDOG_STCTRLH_ALLOWUPDATE     (1 << 4)  /* Bit 4:  Enables updates to watchdog */
#define WDOG_STCTRLH_DBGEN           (1 << 5)  /* Bit 5:  Enables or disables WDOG in Debug mode */
#define WDOG_STCTRLH_STOPEN          (1 << 6)  /* Bit 6:  Enables or disables WDOG in stop mode */
#define WDOG_STCTRLH_WAITEN          (1 << 7)  /* Bit 7:  Enables or disables WDOG in wait mode */
#ifndef KINETIS_K64
#  define WDOG_STCTRLH_STNDBYEN      (1 << 8)  /* Bit 8:  Enables or disables WDOG in Standby mode */
#endif
                                               /* Bit 9:  Reserved */
#define WDOG_STCTRLH_TESTWDOG        (1 << 10) /* Bit 10: Selects functional test mode */
#define WDOG_STCTRLH_TESTSEL         (1 << 11) /* Bit 11: Selects the test to be run */
#define WDOG_STCTRLH_BYTESEL_SHIFT   (12)      /* Bits 12-13: Selects the byte in test mode */
#define WDOG_STCTRLH_BYTESEL_MASK    (3 << WDOG_STCTRLH_BYTESEL_SHIFT)
#  define WDOG_STCTRLH_BYTESEL_BYTE0 (0 << WDOG_STCTRLH_BYTESEL_SHIFT) /* Byte 0 selected */
#  define WDOG_STCTRLH_BYTESEL_BYTE1 (1 << WDOG_STCTRLH_BYTESEL_SHIFT) /* Byte 1 selected */
#  define WDOG_STCTRLH_BYTESEL_BYTE2 (2 << WDOG_STCTRLH_BYTESEL_SHIFT) /* Byte 2 selected */
#  define WDOG_STCTRLH_BYTESEL_BYTE3 (3 << WDOG_STCTRLH_BYTESEL_SHIFT) /* Byte 3 selected */

#define WDOG_STCTRLH_DISTESTWDOG     (1 << 14) /* Bit 14: Disable WDOG�s functional test mode */
                                               /* Bit 15: Reserved */

/* Watchdog Status and Control Register Low (16-bit) */

#define WDOG_STCTRLL_INTFLG          (1 << 15) /* Bit 15: Interrupt flag */
                                               /* Bits 0-14: Reserved */

/* Watchdog Time-out Value Register High/Low (16-bit timeout values) */

/* Watchdog Window Register High/Low (16-bit window values) */

/* Watchdog Refresh Register (16-bit, 0xa602 followed by 0xb480) */

/* Watchdog Unlock Register (16-bit, 0xc520 followed by 0xd928) */

/* Watchdog Timer Output Register High/Low (16-bit timer values) */

/* Watchdog Reset Count Register (16-bit reset count) */

/* Watchdog Prescaler Register (16-bit) */

                                               /* Bits 0-7: Reserved */
#define WDOG_PRESC_PRESCVAL_SHIFT    (8)       /* Bits 8-10: Watchdog clock source prescaler */
#define WDOG_PRESC_PRESCVAL_MASK     (7 << WDOG_PRESC_PRESCVAL_SHIFT)
                                               /* Bits 11-15: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_WDOG_H */
