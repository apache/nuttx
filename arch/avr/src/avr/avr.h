/****************************************************************************
 * arch/avr/src/avr/avr.h
 *
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#ifndef __ARCH_AVR_SRC_AVR_AVR_H
#define __ARCH_AVR_SRC_AVR_AVR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <sys/types.h>
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros to handle saving and restore interrupt state.  The state is copied
 * from the stack to the TCB, but only a referenced is passed to get the
 * state from the TCB.
 */

#define up_savestate(regs)    up_copystate(regs, (uint8_t*)g_current_regs)
#define up_restorestate(regs) (g_current_regs = regs)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* This holds a references to the current interrupt level register storage
 * structure.  If is non-NULL only during interrupt processing.
 */

extern volatile uint8_t *g_current_regs;

/* This is the beginning of heap as provided from up_head.S. This is the first
 * address in DRAM after the loaded program+bss+idle stack.  The end of the
 * heap is CONFIG_RAM_END
 */

extern uint16_t g_idle_topstack;

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct spi_dev_s; /* Forward references */

/************************************************************************************
 * Name:  up_copystate
 *
 * Description:
 *   Copy the contents of a register state save structure from one location to
 *   another.
 *
 ************************************************************************************/

void up_copystate(uint8_t *dest, uint8_t *src);

/************************************************************************************
 * Name:  up_fullcontextrestore
 *
 * Description:
 *   Restore the full context of a saved thread/task.
 *
 ************************************************************************************/

void up_fullcontextrestore(uint8_t *restoreregs) noreturn_function;

/************************************************************************************
 * Name:  up_switchcontext
 *
 * Description:
 *   Switch from one thread/task context to another.
 *
 ************************************************************************************/

void up_switchcontext(uint8_t *saveregs, uint8_t *restoreregs);

/************************************************************************************
 * Name:  up_doirq
 *
 * Description:
 *   Dispatch an interrupt.
 *
 ************************************************************************************/

uint8_t *up_doirq(uint8_t irq, uint8_t *regs);

/****************************************************************************
 * Name: avr_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *avr_spibus_initialize(int port);

/************************************************************************************
 * Name:  avr_spiselect, avr_spitatus, and avr_spicmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   including avr_spibus_initialize()) are provided by common AVR logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in <arch>_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide avr_spiselect() and avr_spistatus() functions in your board-specific
 *      logic.  These functions will perform chip selection and status operations
 *      using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide the
 *      avr_spicmddata() function in your board-specific logic.  This functions will
 *      perform cmd/data selection operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a call to at90usb_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by avr_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling  mmcsd_spislotinitialize(),
 *      for example, will bind the SPI driver to the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_AVR_SPI
void  avr_spiselect(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t avr_spistatus(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int avr_spicmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVR_AVR_H */
