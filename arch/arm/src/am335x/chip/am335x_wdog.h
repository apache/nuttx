/************************************************************************************
 * arch/arm/src/am335x/chip/am335x_wdog.h
 *
 *   Copyright (C) 2019 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_AM335X_CHIP_AM335X_WDOG_H
#define __ARCH_ARM_SRC_AM335X_CHIP_AM335X_WDOG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/am335x_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AM335X_WDT_WIDR_OFFSET          0x0000  /* Watchdog Identification Register */
#define AM335X_WDT_WDSC_OFFSET          0x0010  /* Watchdog System Control Register */
#define AM335X_WDT_WDST_OFFSET          0x0014  /* Watchdog Status Register */
#define AM335X_WDT_WISR_OFFSET          0x0018  /* Watchdog Interrupt Status Register */
#define AM335X_WDT_WIER_OFFSET          0x001C  /* Watchdog Interrupt Enable Register */
#define AM335X_WDT_WCLR_OFFSET          0x0024  /* Watchdog Control Register */
#define AM335X_WDT_WCRR_OFFSET          0x0028  /* Watchdog Counter Register */
#define AM335X_WDT_WLDR_OFFSET          0x002C  /* Watchdog Load Register */
#define AM335X_WDT_WTGR_OFFSET          0x0030  /* Watchdog Trigger Register  */
#define AM335X_WDT_WWPS_OFFSET          0x0034  /* Watchdog Write Posting Bits Register */
#define AM335X_WDT_WDLY_OFFSET          0x0044  /* Watchdog Delay Configuration Register */
#define AM335X_WDT_WSPR_OFFSET          0x0048  /* Watchdog Start/Stop Register */
#define AM335X_WDT_WIRQSTATRAW_OFFSET   0x0054  /* Watchdog Raw Interrupt Status Register */
#define AM335X_WDT_WIRQSTAT_OFFSET      0x0058  /* Watchdog Interrupt Status Register */
#define AM335X_WDT_WIRQENSET_OFFSET     0x005C  /* Watchdog Interrupt Enable Set Register */
#define AM335X_WDT_WIRQENCLR_OFFSET     0x0060  /* Watchdog Interrupt Enable Clear Register */

/* Register addresses ***************************************************************/

#define AM335X_WDT_WIDR                 (AM335X_WDT1_VADDR + AM335X_WDT_WIDR_OFFSET)
#define AM335X_WDT_WDSC                 (AM335X_WDT1_VADDR + AM335X_WDT_WDSC_OFFSET)
#define AM335X_WDT_WDST                 (AM335X_WDT1_VADDR + AM335X_WDT_WDST_OFFSET)
#define AM335X_WDT_WISR                 (AM335X_WDT1_VADDR + AM335X_WDT_WISR_OFFSET)
#define AM335X_WDT_WIER                 (AM335X_WDT1_VADDR + AM335X_WDT_WIER_OFFSET)
#define AM335X_WDT_WCLR                 (AM335X_WDT1_VADDR + AM335X_WDT_WCLR_OFFSET)
#define AM335X_WDT_WCRR                 (AM335X_WDT1_VADDR + AM335X_WDT_WCRR_OFFSET)
#define AM335X_WDT_WLDR                 (AM335X_WDT1_VADDR + AM335X_WDT_WLDR_OFFSET)
#define AM335X_WDT_WTGR                 (AM335X_WDT1_VADDR + AM335X_WDT_WTGR_OFFSET)
#define AM335X_WDT_WWPS                 (AM335X_WDT1_VADDR + AM335X_WDT_WWPS_OFFSET)
#define AM335X_WDT_WDLY                 (AM335X_WDT1_VADDR + AM335X_WDT_WDLY_OFFSET)
#define AM335X_WDT_WSPR                 (AM335X_WDT1_VADDR + AM335X_WDT_WSPR_OFFSET)
#define AM335X_WDT_WIRQSTATRAW          (AM335X_WDT1_VADDR + AM335X_WDT_WIRQSTATRAW_OFFSET)
#define AM335X_WDT_WIRQSTAT             (AM335X_WDT1_VADDR + AM335X_WDT_WIRQSTAT_OFFSET)
#define AM335X_WDT_WIRQENSET            (AM335X_WDT1_VADDR + AM335X_WDT_WIRQENSET_OFFSET)
#define AM335X_WDT_WIRQENCLR            (AM335X_WDT1_VADDR + AM335X_WDT_WIRQENCLR_OFFSET)

/* Register bit definitions *********************************************************/

/* Watchdog System Control Register */

#define WDT_WDSC_SOFTRESET              (1 << 1)  /* Bit 1:  Watchdog Software Reset */
#define WDT_WDSC_IDLEMODE_SHIFT         (3)  /* Bit 3-4:  Watchdog Idle Mode */
#  define WDT_WDSC_IDLEMODE_FORCE       (0 << WDT_WDSC_IDLEMODE_SHIFT)  /* Force-idle Mode */
#  define WDT_WDSC_IDLEMODE_NO          (1 << WDT_WDSC_IDLEMODE_SHIFT)  /* No-idle Mode */
#  define WDT_WDSC_IDLEMODE_SMART       (2 << WDT_WDSC_IDLEMODE_SHIFT)  /* Smart-idle Mode */
#  define WDT_WDSC_IDLEMODE_SMART_WKUP  (3 << WDT_WDSC_IDLEMODE_SHIFT)  /* Smart-idle Wakeup-capable Mode */
#define WDT_WDSC_EMUFREE                (1 << 5)  /* Bit 5:  Watchdog DEBUG Disable */

/* Watchdog Status Register */

#define WDT_WDST_RESETDONE              (1 << 0)  /* Bit 0:  Watchdog Reset Completed */

/* Watchdog Interrupt Registers */

#define WDT_IRQ_FLAG_OVERFLOW           (1 << 0)  /* Bit 0:  Overflow Interrupt Pending */
#define WDT_IRQ_FLAG_DELAY              (1 << 1)  /* Bit 1:  Delay Interrupt Pending */

/* Watchdog Control Register */

#define WDT_WCLR_PTV_SHIFT              (2)  /* Bits 2-4:  Prescaler Value */
#define WDT_WCLR_PTV_MASK               (7 << WDT_WCLR_PTV_SHIFT)
#  define WDT_WCLR_PTV(n)               ((uint32_t)(n) << WDT_WCLR_PTV_SHIFT)
#define WDT_WCLR_PRE_ENABLE             (1 << 5)  /* Bit 5:  Prescaler Enabled */

/* Watchdog Write Posting Bits Register */

#define WDT_WWPS_W_PEND_WCLR            (1 << 0)  /* Bit 0:  Write Pending for Register WCLR */
#define WDT_WWPS_W_PEND_WCRR            (1 << 1)  /* Bit 1:  Write pending for register WCRR */
#define WDT_WWPS_W_PEND_WLDR            (1 << 2)  /* Bit 2:  Write pending for register WLDR */
#define WDT_WWPS_W_PEND_WTGR            (1 << 3)  /* Bit 3:  Write pending for register WTGR */
#define WDT_WWPS_W_PEND_WSPR            (1 << 4)  /* Bit 4:  Write pending for register WSPR */
#define WDT_WWPS_W_PEND_WDLY            (1 << 5)  /* Bit 5:  Write pending for register WDLY */

/* Watchdog Start/Stop Register */

#define WDT_WSPR_START_FEED_A           (0x0000bbbb)
#define WDT_WSPR_START_FEED_B           (0x00004444)
#define WDT_WSPR_STOP_FEED_A            (0x0000aaaa)
#define WDT_WSPR_STOP_FEED_B            (0x00005555)

#endif /* __ARCH_ARM_SRC_AM335X_CHIP_AM335X_WDOG_H */

