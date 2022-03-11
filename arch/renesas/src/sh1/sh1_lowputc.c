/****************************************************************************
 * arch/renesas/src/sh1/sh1_lowputc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Is there a serial console? */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_SH1_SCI0)
#  define HAVE_CONSOLE 1
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_SH1_SCI1)
#  define HAVE_CONSOLE 1
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#else
#  if defined(CONFIG_SCI0_SERIAL_CONSOLE) || defined(CONFIG_SCI1_SERIAL_CONSOLE)
#    error "Serial console selected, but corresponding SCI not enabled"
#  endif
#  undef HAVE_CONSOLE
#endif

/* Select UART parameters for the selected console */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#  define SH1_SCI_BASE     SH1_SCI0_BASE
#  define SH1_SCI_BAUD     CONFIG_SCI0_BAUD
#  define SH1_SCI_BITS     CONFIG_SCI0_BITS
#  define SH1_SCI_PARITY   CONFIG_SCI0_PARITY
#  define SH1_SCI_2STOP    CONFIG_SCI0_2STOP
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#  define SH1_SCI_BASE     SH1_SCI1_BASE
#  define SH1_SCI_BAUD     CONFIG_SCI1_BAUD
#  define SH1_SCI_BITS     CONFIG_SCI1_BITS
#  define SH1_SCI_PARITY   CONFIG_SCI1_PARITY
#  define SH1_SCI_2STOP    CONFIG_SCI1_2STOP
#else
#  error "No CONFIG_SCIn_SERIAL_CONSOLE Setting"
#endif

/* Get mode setting */

#if SH1_SCI_BITS == 7
#  define SH1_SMR_MODE SH1_SCISMR_CHR
#elif SH1_SCI_BITS == 8
#  define SH1_SMR_MODE (0)
#else
#  error "Number of bits not supported"
#endif

#if SH1_SCI_PARITY == 0
#  define SH1_SMR_PARITY (0)
#elif SH1_SCI_PARITY == 1
#  define SH1_SMR_PARITY (SH1_SCISMR_PE|SH1_SCISMR_OE)
#elif SH1_SCI_PARITY == 2
#  define SH1_SMR_PARITY SH1_SCISMR_PE
#else
#  error "Invalid parity selection"
#endif

#if SH1_SCI_2STOP != 0
#  define SH1_SMR_STOP SH1_SCISMR_STOP
#else
#  define SH1_SMR_STOP (0)
#endif

/* The full SMR setting also includes internal clocking with no divisor,
 * aysnchronous operation and multiprocessor disabled:
 */

#define SH1_SMR_VALUE (SH1_SMR_MODE|SH1_SMR_PARITY|SH1_SMR_STOP)

/* Clocking *****************************************************************/

/* The calculation of the BRR to achieve the desired BAUD is given by the
 * following formula:
 *
 *   brr = (f/(64*2**(2n-1)*b))-1
 *
 * Where:
 *
 *   b = bit rate
 *   f = frequency (Hz)
 *   n = divider setting (0, 1, 2, 3)
 *
 * For n == 0, this simplifies to:
 *
 *   brr = (f/(32*b))-1
 *
 * For example, if the processor is clocked at 10 MHz and 9600 is the
 * desired BAUD:
 *
 *   brr = 10,000,000 / (32 * 9600) - 1 = 31.552 (or 32 after rounding)
 */

#define SH1_DIVISOR (32 * SH1_SCI_BAUD)
#define SH1_BRR     (((SH1_CLOCK + (SH1_DIVISOR/2))/SH1_DIVISOR)-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return TRUE of the Transmit Data Register is empty
 *
 ****************************************************************************/

#ifdef HAVE_CONSOLE
static inline int up_txready(void)
{
  /* Check the TDRE bit in the SSR.  1=TDR is empty */

  return ((getreg8(SH1_SCI_BASE + SH1_SCI_SSR_OFFSET) &
           SH1_SCISSR_TDRE) != 0);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  uint8_t ssr;

  /* Wait until the TDR is available */

  while (!up_txready());

  /* Write the data to the TDR */

  putreg8(ch, SH1_SCI_BASE + SH1_SCI_TDR_OFFSET);

  /* Clear the TDRE bit in the SSR */

  ssr  = getreg8(SH1_SCI_BASE + SH1_SCI_SSR_OFFSET);
  ssr &= ~SH1_SCISSR_TDRE;
  putreg8(ssr, SH1_SCI_BASE + SH1_SCI_SSR_OFFSET);
#endif
}

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void up_lowsetup(void)
{
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_SCI_CONFIG)
  uint8_t scr;

  /* Disable the transmitter and receiver */

  scr  = getreg8(SH1_SCI_BASE + SH1_SCI_SCR_OFFSET);
  scr &= ~(SH1_SCISCR_TE | SH1_SCISCR_RE);
  putreg8(scr, SH1_SCI_BASE + SH1_SCI_SCR_OFFSET);

  /* Set communication to be asynchronous with the configured number of data
   * bits, parity, and stop bits.  Use the internal clock (undivided)
   */

  putreg8(SH1_SMR_VALUE, SH1_SCI_BASE + SH1_SCI_SMR_OFFSET);

  /* Set the baud based on the configured console baud and configured
   * system clock.
   */

  putreg8(SH1_BRR, SH1_SCI_BASE + SH1_SCI_BRR_OFFSET);

  /* Select the internal clock source as input */

  scr &= ~SH1_SCISCR_CKEMASK;
  putreg8(scr, SH1_SCI_BASE + SH1_SCI_SCR_OFFSET);

  /* Wait a bit for the clocking to settle */

  up_udelay(100);

  /* Then enable the transmitter and receiver */

  scr |= (SH1_SCISCR_TE | SH1_SCISCR_RE);
  putreg8(scr, SH1_SCI_BASE + SH1_SCI_SCR_OFFSET);
#endif
}
