/****************************************************************************
 * arch/avr/src/avr/avr.h
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
/* This is the beginning of heap as provided from up_head.S. This is the
 * first address in DRAM after the loaded program+bss+idle stack.  The end
 * of the heap is CONFIG_RAM_END
 */

extern uint16_t g_idle_topstack;

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct spi_dev_s; /* Forward references */

/****************************************************************************
 * Name:  up_copystate
 *
 * Description:
 *   Copy the contents of a register state save structure from one location
 *   to another.
 *
 ****************************************************************************/

void up_copystate(uint8_t *dest, uint8_t *src);

/****************************************************************************
 * Name:  up_fullcontextrestore
 *
 * Description:
 *   Restore the full context of a saved thread/task.
 *
 ****************************************************************************/

void up_fullcontextrestore(uint8_t *restoreregs) noreturn_function;

/****************************************************************************
 * Name:  up_switchcontext
 *
 * Description:
 *   Switch from one thread/task context to another.
 *
 ****************************************************************************/

void up_switchcontext(uint8_t *saveregs, uint8_t *restoreregs);

/****************************************************************************
 * Name:  up_doirq
 *
 * Description:
 *   Dispatch an interrupt.
 *
 ****************************************************************************/

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

/****************************************************************************
 * Name:  avr_spiselect, avr_spitatus, and avr_spicmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They
 *   are implementations of the select, status, and cmddata methods of the
 *   SPI interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods including avr_spibus_initialize()) are provided by
 *   common AVR logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in <arch>_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide avr_spiselect() and avr_spistatus() functions in your
 *      board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is
 *      configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      the avr_spicmddata() function in your board-specific logic.  This
 *      functions will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to at90usb_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by avr_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_SPI
void  avr_spiselect(FAR struct spi_dev_s *dev,
                    uint32_t devid, bool selected);
uint8_t avr_spistatus(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int avr_spicmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVR_AVR_H */
