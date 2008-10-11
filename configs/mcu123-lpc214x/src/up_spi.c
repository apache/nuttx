/****************************************************************************
 * config/mcu123-lpc214x/src/up_spi.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * This logic emulates the Prolific PL2303 serial/USB converter
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

/****************************************************************************
 * One the mcu123.com lpc214x board, the MMC slot is connect via SPI with the
 * following SPI mode pinout:
 *
 *   1 CS: Chip select (low) - SSEL1      5 SCLK: Clock         - SCK1
 *   2 DI: Data input        - MOSI1      6 Vss2: Supply Voltage- GRND
 *   3 Vss: Supply Voltage   - GRND       7 DO: Data Output     - MISO1
 *   4 Vdd: Power Supply     - Vcc        8 - N/C
 *
 * The LPC214x supports one SPI port (SPI0) and one SSP port (SPI1).  SPI1
 * is used to interface with the MMC connect
 *
 *   SCK1  - pin 47, P0.17/CAP1.2/SCK1/MAT1.2
 *   MISO1 - pin 53, P0.18/CAP1.3/MISO1/MAT1.3
 *   MOSI1 - pin 54, P0.19/MAT1.2/MOSI1/CAP1.2
 *   SSEL1 - pin 55, P0.20/MAT1.3/SSEL1/EINT3
 *
 * SPI0 is available on the mcu123.com board (pins 27, 29, 30, and 31).
 * Pin 27 is dedicated to a chip select, pins 30 and 31 connect to keys, nd
 * pin 29 is unconnected.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi.h>

#include <arch/board/board.h>
#include <nuttx/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "lpc214x_power.h"
#include "lpc214x_pinsel.h"
#include "lpc214x_spi.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define LPC214X_CCLKFREQ  (LPC214X_FOSC*LPC214X_PLL_M)
#define LPC214X_PCLKFREQ  (LPC214X_CCLKFREQ/LPC214X_APB_DIV)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void   spi_select(boolean select);
static uint32 spi_setclockfrequency(uint32 frequency);
static ubyte  spi_status(void);
static ubyte  spi_sndbyte(ubyte c);
static ubyte  spi_waitready(void);
static void   spi_sndblock(ubyte *data, int datlen);
static void   spi_recvblock(ubyte *data, int datlen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .select            = spi_select,
  .setclockfrequency = spi_setclockfrequency,
  .status            = spi_status,
  .sndbyte           = spi_sndbyte,
  .waitready         = spi_waitready,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   MMC chip-select control
 *
 ****************************************************************************/

void spi_select(boolean select)
{
  uint32 bit = 1 << 20;

  if (select)
    {
      /* Enable chip select (low enables) */

      putreg32(bit, LPC214X_GPIO0_BASE+LPC214X_GPIO_CLR_OFFSET);
    }
  else
    {
      /* Disable chip select (low enables) */

      putreg32(bit, LPC214X_GPIO0_BASE+LPC214X_GPIO_SET_OFFSET);

      /* Wait for the TX FIFO not full indication */

      while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF));
      putreg16(0xff, LPC214X_SPI1_DR);

      /* Then wait until TX FIFO and TX shift buffer are empty */

      while (getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_BSY);
      while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE));

      do
        {
          (void)getreg16(LPC214X_SPI1_DR);
        }
      while (getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE);
    }
}

/****************************************************************************
 * Name: spi_status
 * Description:
 *   Return MMC present + write protect status
 *
 ****************************************************************************/

ubyte spi_status(void)
{
  return SPI_STATUS_PRESENT;
}

/****************************************************************************
 * Name: spi_setclockfrequency
 *
 * Description:
 *   Set SPI clock frequency
 *
 ****************************************************************************/

uint32 spi_setclockfrequency(uint32 frequency)
{
  uint32 divisor = LPC214X_PCLKFREQ / frequency;

  if (divisor < 2)
    {
      divisor = 2;
    }
  else if (divisor > 254)
    {
      divisor = 254;
    }

  divisor = (divisor + 1) & ~1;
  putreg8(divisor, LPC214X_SPI1_CPSR);
  return LPC214X_PCLKFREQ / divisor;
}

/****************************************************************************
 * Name: spi_sndbyte
 *
 * Description:
 *   Transfer one byte to the SPI
 *
 ****************************************************************************/

ubyte spi_sndbyte(ubyte c)
{
  /* Wait while the TX FIFO is full */

  while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF));

  /* Send the byte */

  putreg16(c, LPC214X_SPI1_DR);

  /* Wait for the RX FIFO not empty */

  while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE));

  /* Return the value from the RX FIFO */

  return (ubyte)getreg16(LPC214X_SPI1_DR);
}

/****************************************************************************
 * Name: spi_waitready
 ****************************************************************************/

ubyte spi_waitready(void)
{
  ubyte ret;

  do
    {
      /* Write 0xff to the data register */

      putreg16(0xff, LPC214X_SPI1_DR);

      /* Wait until the controller reports RX FIFO not empty */

      while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE));

      /* Check by reading back the from the RX FIFO */

      ret = (ubyte)getreg16(LPC214X_SPI1_DR);
    }
  while (ret != 0xff);

  return ret;
}

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Read a block of bytes from SPI
 *
 *************************************************************************/

void spi_sndblock(ubyte *data, int datlen)
{
  /* Loop while thre are bytes remaining to be sent */

  while (datlen > 0)
    {
      /* While the TX FIFO is not full and there are bytes left to send */

      while ((getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF) && datlen)
        {
          /* Send the data */

          putreg16((uint16)*data, LPC214X_SPI1_DR);
          data++;
          datlen--;
        }
    }

  /* Then read from the FIFO while the RX FIFO is not empty or the TX FIFO
   * is not empty.
   */

  while ((getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE) ||
        !(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TFE))
    {
      (void)getreg16(LPC214X_SPI1_DR);
    }
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of bytes from SPI
 *
 *************************************************************************/

static void spi_recvblock(ubyte *data, int datlen)
{
  uint32 fifobytes = 0;

  /* While there is remaining to be sent (and no error has occurred */

  while (datlen || fifobytes)
    {
      /* Fill the transmit FIFO with 0xff...
       * Write 0xff to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      while ((getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF) &&
             (fifobytes < LPC214X_SPI1_FIFOSZ) && datlen)
        {
          putreg16(0xff, LPC214X_SPI1_DR);
          --datlen;
          ++fifobytes;
        }

      /* Now, read the RX data from the FIFO while the RX FIFO is not empty */

      while (getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE)
        {
          *data++ = (ubyte)getreg16(LPC214X_SPI1_DR);
          --fifobytes;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 ****************************************************************************/

void up_spiinitialize(int port)
{
  uint32 regval32;
  ubyte regval8;
  int i;

  /* Configure multiplexed pins as connected on the mcu123.com board:
   *
   *   PINSEL1 P0.17/CAP1.2/SCK1/MAT1.2  Bits 2-3=10 for SCK1
   *   PINSEL1 P0.18/CAP1.3/MISO1/MAT1.3 Bits 4-5=10 for MISO1
   *   PINSEL1 P0.19/MAT1.2/MOSI1/CAP1.2 Bits 6-7=10 for MOSI1
   *   PINSEL1 P0.20/MAT1.3/SSEL1/EINT3  Bits 8-9=10 for P0.20 (we'll control it via GPIO)
   */

  regval32  = getreg32(LPC214X_PINSEL1);
  regval32 &= ~(LPC214X_PINSEL1_P017_MASK|LPC214X_PINSEL1_P018_MASK|
                LPC214X_PINSEL1_P019_MASK|LPC214X_PINSEL1_P020_MASK);
  regval32 |= (LPC214X_PINSEL1_P017_SCK1|LPC214X_PINSEL1_P018_MISO1|
                LPC214X_PINSEL1_P019_MOSI1|LPC214X_PINSEL1_P020_GPIO);
  putreg32(regval32, LPC214X_PINSEL1);

  /* Disable chip select using P0.20 (SSEL1)  (low enables) */

  regval32 = 1 << 20;
  putreg32(regval32, LPC214X_GPIO0_BASE+LPC214X_GPIO_SET_OFFSET);
  regval32 |= getreg32(LPC214X_GPIO0_BASE+LPC214X_GPIO_DIR_OFFSET);
  putreg32(regval32, LPC214X_GPIO0_BASE+LPC214X_GPIO_DIR_OFFSET);

  /* Enable peripheral clocking to SPI1 */

  regval32  = getreg32(LPC214X_PCON_PCONP);
  regval32 |= LPC214X_PCONP_PCSPI1;
  putreg32(regval32, LPC214X_PCON_PCONP);

  /* Configure 8-bit SPI mode */

  putreg16(LPC214X_SPI1CR0_DSS8BIT|LPC214X_SPI1CR0_FRFSPI, LPC214X_SPI1_CR0);

  /* Disable the SSP and all interrupts (we'll poll for all data) */

  putreg8(0, LPC214X_SPI1_CR1);
  putreg8(0, LPC214X_SPI1_IMSC);

  /* Set the initial clock frequency for indentification mode < 400kHz */

  spi_setclockfrequency(400000);

  /* Enable the SPI */

  regval8 = getreg8(LPC214X_SPI1_CR1);
  putreg8(regval8 | LPC214X_SPI1CR1_SSE, LPC214X_SPI1_CR1);

  for (i = 0; i < 8; i++)
    {
      (void)getreg16(LPC214X_SPI1_DR);
    }
}

/****************************************************************************
 * Name: up_spigetvtable
 *
 * Description:
 *   Return the vtable for the selected SPI port
 *
 ****************************************************************************/

FAR const struct spi_ops_s *up_spigetvtable(int port)
{
  return &g_spiops;
}

