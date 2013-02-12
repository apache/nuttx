/************************************************************************************
 * configs/olimex-lpc1766stk/src/lpc1766stk_internal.h
 * arch/arm/src/board/lpc1766stk_internal.n
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

#ifndef _CONFIGS_OLIMEX_LPC1766STK_SRC_LPC1766STK_INTERNAL_H
#define _CONFIGS_OLIMEX_LPC1766STK_SRC_LPC1766STK_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* LPC1766-STK GPIO Pin Definitions *************************************************/
/* GPIO P2[21] connects to the Ready/Busy pin of the NAND part.  We need to
 * reconfigure this pin as normal GPIO input if NAND is used.
 */

#define GPIO_NAND_RB (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN21)

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
 * Name: lpc17_sspinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Olimex LPC1766-STK board.
 *
 ************************************************************************************/

void weak_function lpc17_sspinitialize(void);

/************************************************************************************
 * Name: lpc17_sdram_initialize
 *
 * Description:
 *   Initialize SDRAM
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_EMC
#ifdef CONFIG_LPC17_EMC_SDRAM
void lpc17_sdram_initialize(void);
#endif

/************************************************************************************
 * Name: lpc17_nor_initialize
 *
 * Description:
 *   Initialize NOR FLASH
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_EMC_NOR
void lpc17_nor_initialize(void);
#endif

/************************************************************************************
 * Name: lpc17_nand_initialize
 *
 * Description:
 *   Initialize NAND FLASH
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_EMC_NAND
void lpc17_nand_initialize(void);
#endif
#endif /* CONFIG_LPC17_EMC */

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_OLIMEX_LPC1766STK_SRC_LPC1766STK_INTERNAL_H */

