/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_lowputc.c
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
#include "rx65n_definitions.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Is there a serial console? */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI0)
#  define HAVE_CONSOLE 1
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI2)
#  define HAVE_CONSOLE 1
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#else
#  if defined(CONFIG_SCI0_SERIAL_CONSOLE) || defined(CONFIG_SCI2_SERIAL_CONSOLE)
#    error "Serial console selected, but corresponding SCI not enabled"
#  endif
#  undef HAVE_CONSOLE
#endif

/* Select UART parameters for the selected console */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX65N_SCI0_BASE
#  define RX_SCI_BAUD     CONFIG_SCI0_BAUD
#  define RX_SCI_BITS     CONFIG_SCI0_BITS
#  define RX_SCI_PARITY   CONFIG_SCI0_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI0_2STOP
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX65N_SCI1_BASE
#  define RX_SCI_BAUD     CONFIG_SCI1_BAUD
#  define RX_SCI_BITS     CONFIG_SCI1_BITS
#  define RX_SCI_PARITY   CONFIG_SCI1_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI1_2STOP
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX65N_SCI2_BASE
#  define RX_SCI_BAUD     CONFIG_SCI2_BAUD
#  define RX_SCI_BITS     CONFIG_SCI2_BITS
#  define RX_SCI_PARITY   CONFIG_SCI2_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI2_2STOP
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX65N_SCI3_BASE
#  define RX_SCI_BAUD     CONFIG_SCI3_BAUD
#  define RX_SCI_BITS     CONFIG_SCI3_BITS
#  define RX_SCI_PARITY   CONFIG_SCI3_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI3_2STOP
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX65N_SCI4_BASE
#  define RX_SCI_BAUD     CONFIG_SCI4_BAUD
#  define RX_SCI_BITS     CONFIG_SCI4_BITS
#  define RX_SCI_PARITY   CONFIG_SCI4_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI4_2STOP
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX65N_SCI5_BASE
#  define RX_SCI_BAUD     CONFIG_SCI5_BAUD
#  define RX_SCI_BITS     CONFIG_SCI5_BITS
#  define RX_SCI_PARITY   CONFIG_SCI5_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI5_2STOP
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX65N_SCI6_BASE
#  define RX_SCI_BAUD     CONFIG_SCI6_BAUD
#  define RX_SCI_BITS     CONFIG_SCI6_BITS
#  define RX_SCI_PARITY   CONFIG_SCI6_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI6_2STOP
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX65N_SCI7_BASE
#  define RX_SCI_BAUD     CONFIG_SCI7_BAUD
#  define RX_SCI_BITS     CONFIG_SCI7_BITS
#  define RX_SCI_PARITY   CONFIG_SCI7_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI7_2STOP
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX65N_SCI8_BASE
#  define RX_SCI_BAUD     CONFIG_SCI8_BAUD
#  define RX_SCI_BITS     CONFIG_SCI8_BITS
#  define RX_SCI_PARITY   CONFIG_SCI8_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI8_2STOP
#else
#  error "No CONFIG_SCIn_SERIAL_CONSOLE Setting"
#endif

/* Get mode setting */

#if RX_SCI_BITS == 7
#  define RX_SMR_MODE RX_SCISMR_CHR
#elif RX_SCI_BITS == 8
#  define RX_SMR_MODE (0)
#else
#  define RX_SMR_MODE (0)
#endif

#if RX_SCI_PARITY == 0
#  define RX_SMR_PARITY (0)
#elif RX_SCI_PARITY == 1
#  define RX_SMR_PARITY (RX_SCISMR_PE|RX_SCISMR_OE)
#elif RX_SCI_PARITY == 2
#  define RX_SMR_PARITY RX_SCISMR_PE
#else
#  define RX_SMR_PARITY (0)
#endif

#if RX_SCI_2STOP != 0
#  define RX_SMR_STOP RX_SCISMR_STOP
#else
#  define RX_SMR_STOP (0)
#endif

/* The full SMR setting also includes internal clocking with no divisor,
 * aysnchronous operation and multiprocessor disabled:
 */

#define RX_SMR_VALUE (RX_SMR_MODE|RX_SMR_PARITY|RX_SMR_STOP)

/* Clocking *****************************************************************/

#define RX_DIVISOR (8 * RX_SCI_BAUD)
#define RX_BRR     ((RX_PCLKB / RX_DIVISOR) - 1)

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

  return ((getreg8(RX_SCI_BASE + RX_SCI_SSR_OFFSET) & RX_SCISSR_TDRE) != 0);
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

  putreg8(ch, RX_SCI_BASE + RX_SCI_TDR_OFFSET);

  /* Clear the TDRE bit in the SSR */

  ssr  = getreg8(RX_SCI_BASE + RX_SCI_SSR_OFFSET);
  ssr &= ~RX_SCISSR_TDRE;
  putreg8(ssr, RX_SCI_BASE + RX_SCI_SSR_OFFSET);
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

  scr  = getreg8(RX_SCI_BASE + RX_SCI_SCR_OFFSET);
  scr &= ~(RX_SCISCR_TE | RX_SCISCR_RE);
  putreg8(scr, RX_SCI_BASE + RX_SCI_SCR_OFFSET);

  /* Set communication to be asynchronous with the configured number of data
   * bits, parity, and stop bits.  Use the internal clock (undivided)
   */

  putreg8(RX_SMR_VALUE, RX_SCI_BASE + RX_SCI_SMR_OFFSET);

  /* Set the baud based on the configured console baud and configured
   * system clock.
   */

  putreg8(RX_BRR, RX_SCI_BASE + RX_SCI_BRR_OFFSET);

  /* Select the internal clock source as input */

  scr &= ~RX_SCISCR_CKEMASK;
  putreg8(scr, RX_SCI_BASE + RX_SCI_SCR_OFFSET);

  /* Then enable the transmitter and receiver */

  scr |= (RX_SCISCR_TE | RX_SCISCR_RE);
  putreg8(scr, RX_SCI_BASE + RX_SCI_SCR_OFFSET);
#endif
}
