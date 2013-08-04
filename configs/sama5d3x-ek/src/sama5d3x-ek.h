/************************************************************************************
 * configs/sama5d3x-ek/src/sama5d3x-ek.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __CONFIGS_SAMA5D3X_EK_SRC_SAMA5D3X_EK_H
#define __CONFIGS_SAMA5D3X_EK_SRC_SAMA5D3X_EK_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "chip/sam_pinmap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* LEDs *****************************************************************************/
/* There are two LEDs on the SAMA5D3 series-CM board that can be controlled
 * by software.  A  blue LED is controlled via GPIO pins.  A red LED normally
 * provides an indication that power is supplied to the board but can also
 * be controlled via software.
 *
 *   PE25.  This blue LED is pulled high and is illuminated by pulling PE25
 *   low.
 *
 *   PE24.  The red LED is also pulled high but is driven by a transistor so
 *   that it is illuminated when power is applied even if PE24 is not
 *   configured as an output.  If PE24 is configured as an output, then the
 *   LCD is illuminated by a high output.
 */

#define GPIO_BLUE    (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOE | GPIO_PIN25)
#define GPIO_RED     (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOE | GPIO_PIN24)

/* Buttons **************************************************************************/
/* There are five push button switches on the SAMA5D3X-EK base board:
 *
 *   1. One Reset, board reset (BP1)
 *   2. One Wake up, push button to bring the processor out of low power mode
 *     (BP2)
 *   3. One User momentary Push Button
 *   4. One Disable CS Push Button
 *
 * Only the momentary push button is controllable by software (labeled
 * "PB_USER1" on the board):
 *
 *   - PE27.  Pressing the switch connect PE27 to grounded.  Therefore, PE27
 *     must be pulled high internally.  When the button is pressed the SAMA5
 *     will sense "0" is on PE27.
 */

#define GPIO_USER1   (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                      GPIO_INT_BOTHEDGES | GPIO_PORT_PIOE | GPIO_PIN27)
#define IRQ_USER1    SAM_IRQ_PE27

/* SPI Chip Selects *****************************************************************/
/* Both the Ronetix and Embest versions of the SAMAD3x CPU modules include an
 * Atmel AT25DF321A, 32-megabit, 2.7-volt SPI serial flash.  The SPI
 * connection is as follows:
 *
 *   AT25DF321A      SAMA5
 *   --------------- -----------------------------------------------
 *   SI              PD11 SPI0_MOSI
 *   SO              PD10 SPI0_MIS0
 *   SCK             PD12 SPI0_SPCK
 *   /CS             PD13 via NL17SZ126 if JP1 is closed (See below)
 *
 * JP1 and JP2 seem to related to /CS on the Ronetix board, but the usage is
 * less clear.  For the Embest module, JP1 must be closed to connect /CS to
 * PD13; on the Ronetix schematic, JP11 seems only to bypass a resistor (may
 * not be populated?).  I think closing JP1 is correct in either case.
 */

#define GPIO_AT25_NPCS0 (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                         GPIO_PORT_PIOD | GPIO_PIN13)
#define AT25_CSNUM      0

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select PIO pins for the SAMA4D3x-EK board.
 *
 ************************************************************************************/

#if defined(CONFIG_SAMA5_SPI0) || defined(CONFIG_SAMA5_SPI1)
void weak_function sam_spiinitialize(void);
#endif

/************************************************************************************
 * Name: board_sdram_config
 *
 * Description:
 *   Configures DDR2 (MT47H128M16RT 128MB or, optionally,  MT47H64M16HR)
 *
 *   Per the SAMA5D3x-EK User guide: "Two SDRAM/DDR2 used as main system memory.
 *   MT47H128M16 - 2 Gb - 16 Meg x 16 x 8 banks, the board provides up to 2 Gb on-
 *   board, soldered DDR2 SDRAM. The memory bus is 32 bits wide and operates with
 *   up to 166 MHz."
 *
 *   From the Atmel Code Example:
 *     MT47H64M16HR : 8 Meg x 16 x 8 banks
 *     Refresh count: 8K
 *     Row address: A[12:0] (8K)
 *     Column address A[9:0] (1K)
 *     Bank address BA[2:0] a(24,25) (8)
 *
 *  This logic was taken from Atmel sample code for the SAMA5D3x-EK.
 *
 *  Input Parameters:
 *     devtype - Either DDRAM_MT47H128M16RT or DDRAM_MT47H64M16HR
 *
 *  Assumptions:
 *    The DDR memory regions is configured as strongly ordered memory.  When we
 *    complete initialization of SDRAM and it is ready for use, we will make DRAM
 *    into normal memory.
 *
 ************************************************************************************/

#if defined(CONFIG_SAMA5_DDRCS) && !defined(CONFIG_SAMA5_BOOT_SDRAM)
void sam_sdram_config(void);
#else
#  define board_sdram_config(t)
#endif

/************************************************************************************
 * Name: up_ledinit
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAMA5D3X_EK_SRC_SAMA5D3X_EK_H */

