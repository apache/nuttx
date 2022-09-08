/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_watchdog.h
 *
 * Generated from rp2040.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_WATCHDOG_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_WATCHDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_WATCHDOG_CTRL_OFFSET      0x000000  /* Watchdog control The rst_wdsel register determines which subsystems are reset when the watchdog is triggered. The watchdog can be triggered in software. */
#define RP2040_WATCHDOG_LOAD_OFFSET      0x000004  /* Load the watchdog timer. The maximum setting is 0xffffff which corresponds to 0xffffff / 2 ticks before triggering a watchdog reset (see errata RP2040-E1). */
#define RP2040_WATCHDOG_REASON_OFFSET    0x000008  /* Logs the reason for the last reset. Both bits are zero for the case of a hardware reset. */
#define RP2040_WATCHDOG_SCRATCH0_OFFSET  0x00000c  /* Scratch register. Information persists through soft reset of the chip. */
#define RP2040_WATCHDOG_SCRATCH1_OFFSET  0x000010  /* Scratch register. Information persists through soft reset of the chip. */
#define RP2040_WATCHDOG_SCRATCH2_OFFSET  0x000014  /* Scratch register. Information persists through soft reset of the chip. */
#define RP2040_WATCHDOG_SCRATCH3_OFFSET  0x000018  /* Scratch register. Information persists through soft reset of the chip. */
#define RP2040_WATCHDOG_SCRATCH4_OFFSET  0x00001c  /* Scratch register. Information persists through soft reset of the chip. */
#define RP2040_WATCHDOG_SCRATCH5_OFFSET  0x000020  /* Scratch register. Information persists through soft reset of the chip. */
#define RP2040_WATCHDOG_SCRATCH6_OFFSET  0x000024  /* Scratch register. Information persists through soft reset of the chip. */
#define RP2040_WATCHDOG_SCRATCH7_OFFSET  0x000028  /* Scratch register. Information persists through soft reset of the chip. */
#define RP2040_WATCHDOG_TICK_OFFSET      0x00002c  /* Controls the tick generator */

/* Register definitions *****************************************************/

#define RP2040_WATCHDOG_CTRL       (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_CTRL_OFFSET)
#define RP2040_WATCHDOG_LOAD       (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_LOAD_OFFSET)
#define RP2040_WATCHDOG_REASON     (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_REASON_OFFSET)
#define RP2040_WATCHDOG_SCRATCH0   (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_SCRATCH0_OFFSET)
#define RP2040_WATCHDOG_SCRATCH1   (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_SCRATCH1_OFFSET)
#define RP2040_WATCHDOG_SCRATCH2   (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_SCRATCH2_OFFSET)
#define RP2040_WATCHDOG_SCRATCH3   (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_SCRATCH3_OFFSET)
#define RP2040_WATCHDOG_SCRATCH4   (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_SCRATCH4_OFFSET)
#define RP2040_WATCHDOG_SCRATCH5   (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_SCRATCH5_OFFSET)
#define RP2040_WATCHDOG_SCRATCH6   (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_SCRATCH6_OFFSET)
#define RP2040_WATCHDOG_SCRATCH7   (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_SCRATCH7_OFFSET)
#define RP2040_WATCHDOG_SCRATCH(n) (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_SCRATCH0_OFFSET + 4 * n)
#define RP2040_WATCHDOG_TICK       (RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_TICK_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_WATCHDOG_CTRL_TRIGGER      (1 << 31)   /* Trigger a watchdog reset */
#define RP2040_WATCHDOG_CTRL_ENABLE       (1 << 30)   /* When not enabled the watchdog timer is paused */
#define RP2040_WATCHDOG_CTRL_PAUSE_DBG1   (1 << 26)   /* Pause the watchdog timer when processor 1 is in debug mode */
#define RP2040_WATCHDOG_CTRL_PAUSE_DBG0   (1 << 25)   /* Pause the watchdog timer when processor 0 is in debug mode */
#define RP2040_WATCHDOG_CTRL_PAUSE_JTAG   (1 << 24)   /* Pause the watchdog timer when JTAG is accessing the bus fabric */
#define RP2040_WATCHDOG_CTRL_TIME_MASK    (0xffffff)  /* Indicates the number of ticks / 2 (see errata RP2040-E1) before a watchdog reset will be triggered */

#define RP2040_WATCHDOG_LOAD_MASK         (0xffffff)

#define RP2040_WATCHDOG_REASON_FORCE      (1 << 1)
#define RP2040_WATCHDOG_REASON_TIMER      (1 << 0)

#define RP2040_WATCHDOG_TICK_COUNT_SHIFT  (11)       /* Count down timer: the remaining number clk_tick cycles before the next tick is generated. */
#define RP2040_WATCHDOG_TICK_COUNT_MASK   (0x1ff << RP2040_WATCHDOG_TICK_COUNT_SHIFT)
#define RP2040_WATCHDOG_TICK_RUNNING      (1 << 10)  /* Is the tick generator running? */
#define RP2040_WATCHDOG_TICK_ENABLE       (1 << 9)   /* start / stop tick generation */
#define RP2040_WATCHDOG_TICK_CYCLES_MASK  (0x1ff)    /* Total number of clk_tick cycles before the next tick. */

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_WATCHDOG_H */
