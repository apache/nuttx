/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_wdt.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
