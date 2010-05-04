/****************************************************************************
 * configs/olimex-strp711/src/up_enc28j60.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/*
 * ENC28J60 Module
 *
 * The ENC28J60 module does not come on the Olimex-STR-P711, but this describes
 * how I have connected it:
 *
 * Module CON5     QFN ENC2860 Description
 * --------------- -------------------------------------------------------
 * 1  J8-1 NET CS   5  ~CS    Chip select input pin for SPI interface (active low)
 * 2     2 SCK      4  SCK    Clock in pin for SPI interface
 * 3     3 MOSI     3  SI     Data in pin for SPI interface
 * 4     4 MISO     2  SO     Data out pin for SPI interface
 * 5     5 GND      -- ---    ---
 * 10 J9-1 3V3      -- ---    ---
 * 9     2 WOL      1  ~WOL   Unicast WOL filter
 * 8     3 NET INT  28 ~INT   Interrupt output pin (active low)
 * 7     4 CLKOUT   27 CLKOUT Programmable clock output pin
 * 6     5 NET RST  6  ~RESET Active-low device Reset input
 *
 * For the Olimex STR-P711, the ENC28J60 module is placed on SPI0 and uses
 * P0.3 for CS, P1.4 for an interrupt, and P1.5 as a reset:
 *
 * Module CON5     Olimex STR-P711 Connection
 * --------------- -------------------------------------------------------
 * 1  J8-1 NET CS   SPI0-2     P0.3 output P0.3/S0.SS/I1.SDA
 * 2     2 SCK      SPI0-5     SCLK0       P0.2/S0.SCLK/I1.SCL
 * 3     3 MOSI     SPI0-3     MOSI0       P0.0/S0.MOSI/U3.RX
 * 4     4 MISO     SPI0-4     MISO0       P0.1/S0.MISO/U3.TX
 * 5     5 GND      SPI0-1     GND
 * 10 J9-1 3V3      SPI0-6     3.3V
 * 9     2 WOL      NC
 * 8     3 NET INT  TMR1_EXT-5 P1.4 input  P1.4/T1.ICAPA/T1.EXTCLK
 * 7     4 CLKOUT   NC
 * 6     5 NET RST  TMR1_EXT_4 P1.5 output P1.5/T1.ICAPB
 *
 * UART3, I2C cannot be used with SPI0.  The GPIOs selected for the ENC28J60
 * interrupt conflict with TMR1.
 */
#warning "Need to select differnt interrupt pin.. XTI doesn't support this one"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/spi.h>
#include <nuttx/enc28j60.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "str71x_internal.h"

#ifdef CONFIG_NET_ENC28J60

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* We assume that the ENC28J60 is on SPI0 */

#ifndef CONFIG_STR71X_BSPI0
# error "Need CONFIG_STR71X_BSPI0 in the configuration"
#endif

#ifndef CONFIG_STR71X_XTI
# error "Need CONFIG_STR71X_XTI in the configuration"
#endif

/* UART3, I2C cannot be used with SPI0.  The GPIOs selected for the ENC28J60
 * interrupt conflict with TIM1.
 */

#ifdef CONFIG_STR71X_UART3
# error "CONFIG_STR71X_UART3 cannot be used in this configuration"
#endif

#ifdef CONFIG_STR71X_TIM1
# error "CONFIG_STR71X_TIM1 cannot be used in this configuration"
#endif

/* SPI Assumptions **********************************************************/

#define ENC28J60_SPI_PORTNO 0                   /* On SPI0 */
#define ENC28J60_DEVNO      0                   /* Only one ENC28J60 */
#define ENC28J60_IRQ        STR71X_IRQ_FIRSTXTI /* NEEDED!!!!!!!!!!!!!!!! */

#warning "Eventually need to fix XTI IRQ number!"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_netinitialize
 ****************************************************************************/

void up_netinitialize(void)
{
  FAR struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  spi = up_spiinitialize(ENC28J60_SPI_PORTNO);
  if (!spi)
    {
      nlldbg("Failed to initialize SPI port %d\n", ENC28J60_SPI_PORTNO);
      return;
    }

  /* Configure the XTI for the ENC28J60 interrupt.  */

  ret = str71x_xticonfig(ENC28J60_IRQ, false);
  if (ret < 0)
    {
      nlldbg("Failed configure interrupt for IRQ %d: %d\n", ENC28J60_IRQ, ret);
      return;
    }

  /* Bind the SPI port to the ENC28J60 driver */

  ret = enc_initialize(spi, ENC28J60_DEVNO, ENC28J60_IRQ);
  if (ret < 0)
    {
      nlldbg("Failed to bind SPI port %d ENC28J60 device %d: %d\n",
             ENC28J60_SPI_PORTNO, ENC28J60_DEVNO, ret);
      return;
    }

  nllvdbg("Bound SPI port %d to ENC28J60 device %d\n",
        ENC28J60_SPI_PORTNO, ENC28J60_DEVNO);
}
#endif /* CONFIG_NET_ENC28J60 */
