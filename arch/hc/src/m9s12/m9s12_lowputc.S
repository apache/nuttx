/**************************************************************************
 * arch/hc/src/m9s12/m9s12_lowputc.S
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include "hc_internal.h"
#include "m9s12.h"
#include "m9s12_sci.h"
#include "m9s12_serial.h"

#ifdef CONFIG_HCS12_SERIALMON
#  include "m9s12_flash.h"
#endif

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

#ifdef CONFIG_HCS12_NONBANKED
#  define CALL   jsr
#  define RETURN rts
#else
#  define CALL   call
#  define RETURN rtc
#endif

/* Select SCI parameters for the selected console */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#  define HCS12_CONSOLE_BASE     HCS12_SCI0_BASE
#  define HCS12_CONSOLE_BAUD     CONFIG_SCI0_BAUD
#  define HCS12_CONSOLE_BITS     CONFIG_SCI0_BITS
#  define HCS12_CONSOLE_PARITY   CONFIG_SCI0_PARITY
#  define HCS12_CONSOLE_2STOP    CONFIG_SCI0_2STOP
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#  define HCS12_CONSOLE_BASE     HCS12_SCI1_BASE
#  define HCS12_CONSOLE_BAUD     CONFIG_SCI1_BAUD
#  define HCS12_CONSOLE_BITS     CONFIG_SCI1_BITS
#  define HCS12_CONSOLE_PARITY   CONFIG_SCI1_PARITY
#  define HCS12_CONSOLE_2STOP    CONFIG_SCI1_2STOP
#else
#  warning "No CONFIG_SCIn_SERIAL_CONSOLE Setting"
#endif

/* Selete the SCIBR register value */

#define CONSOLE_SCIBR_VALUE      SCIBR_VALUE(HCS12_CONSOLE_BAUD)

/* Select the SCICR1 register settings */

#if HCS12_CONSOLE_BITS == 9
#  define UART_SCICR1_NBITS      SCI_CR1_M
#elif HCS12_CONSOLE_BITS == 8
#  define UART_SCICR1_NBITS      0
#else
#  warning "Number of bits not supported"
#endif

#if HCS12_CONSOLE_PARITY == 0
#  define UART_SCICR1_PARITY     0
#elif HCS12_CONSOLE_PARITY == 1
#  define UART_SCICR1_PARITY     SCI_CR1_PE
#elif HCS12_CONSOLE_PARITY == 2
#  define UART_SCICR1_PARITY     (SCI_CR1_PE|SCI_CR1_PT)
#else
#  error "ERROR: Invalid parity selection"
#endif

#if HCS12_CONSOLE_2STOP != 0
#  warning "Only a single stop bit supported"
#endif

#define CONSOLE_SCICR_VALUE      (UART_SCICR1_NBITS|UART_SCICR1_PARITY)

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Public Data
 **************************************************************************/

/**************************************************************************
 * Private Data
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_lowsetup
 *
 * Note: This function is called from the boot logic very early in the
 * initialization sequence:  After the stack pointer has been setup, but
 * before .bss has been cleared and .data initialized.
 *
 **************************************************************************/

	.text
	.globl	up_lowsetup
	.type	up_lowsetup, function
up_lowsetup:
#ifdef HAVE_SERIAL_CONSOLE
#ifndef CONFIG_HCS12_SERIALMON

	/* Disable the console */

	ldab	#0
	stab	(HCS12_CONSOLE_BASE+HCS12_SCI_CR1_OFFSET)
	stab	(HCS12_CONSOLE_BASE+HCS12_SCI_CR2_OFFSET)

	/* Set the BAUD pre-scaler value */

	ldab	#(CONSOLE_SCIBR_VALUE >> 8)
	stab	(HCS12_CONSOLE_BASE+HCS12_SCI_BDH_OFFSET)

	ldab	#(CONSOLE_SCIBR_VALUE & 0xff)
	stab	(HCS12_CONSOLE_BASE+HCS12_SCI_BDL_OFFSET)

	/* Set number of bits, parity, stop bits, etc. */

	ldab	#CONSOLE_SCICR_VALUE
	stab	(HCS12_CONSOLE_BASE+HCS12_SCI_CR1_OFFSET)

	/* Enable transmitter and receiver */

	ldab	#(SCI_CR2_RE|SCI_CR2_TE)
	stab	(HCS12_CONSOLE_BASE+HCS12_SCI_CR2_OFFSET)

#endif /* CONFIG_HCS12_SERIALMON */
#endif /* HAVE_SERIAL_CONSOLE */
	RETURN
	.size	up_lowsetup, . - up_lowsetup

/**************************************************************************
 * Name: hc_lowputc
 **************************************************************************/

	.text
	.global	hc_lowputc
	.type	hc_lowputc, function
hc_lowputc:
#ifdef HAVE_SERIAL_CONSOLE
#ifdef CONFIG_HCS12_SERIALMON
	jmp		PutChar
#else
#if HCS12_CONSOLE_BITS == 9
	staa	1, -sp
#endif /* HCS12_CONSOLE_BITS == 9 */

	/* Wait for the transmit data register to be available (TRDE==1) */

.Lwait:
	ldaa	(HCS12_CONSOLE_BASE+HCS12_SCI_SR1_OFFSET)
	bita	#SCI_SR1_TDRE
	bne		.Lwait

	/* Then write the byte to the transmit data register */

#if HCS12_CONSOLE_BITS == 9
	ldaa	1, sp+
	bita	#(0x01)
	bne		.L8bit
	ldaa	#SCI_DRH_T8
	bra		.Lwrdrh
.L8bit:
	ldaa	#0
.Lwrdrh:
	staa	(HCS12_CONSOLE_BASE+HCS12_SCI_DRH_OFFSET)
#endif /* HCS12_CONSOLE_BITS == 9 */

	stab	(HCS12_CONSOLE_BASE+HCS12_SCI_DRL_OFFSET)
	RETURN

#endif /* !CONFIG_HCS12_SERIALMON */
#else
	RETURN
#endif /* HAVE_SERIAL_CONSOLE */
	.size	hc_lowputc, . - hc_lowputc
	.end
