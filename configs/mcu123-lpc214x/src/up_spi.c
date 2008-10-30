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
#include <nuttx/arch.h>
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

static void   spi_select(FAR struct spi_dev_s *dev, boolean selected);
static uint32 spi_setfrequency(FAR struct spi_dev_s *dev, uint32 frequency);
static ubyte  spi_status(FAR struct spi_dev_s *dev);
static ubyte  spi_sndbyte(FAR struct spi_dev_s *dev, ubyte ch);
static void   spi_sndblock(FAR struct spi_dev_s *dev, FAR const ubyte *buffer, size_t buflen);
static void   spi_recvblock(FAR struct spi_dev_s *dev, FAR ubyte *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .status            = spi_status,
  .sndbyte           = spi_sndbyte,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
};

static struct spi_dev_s g_spidev = { &g_spiops };

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
 *   Enable/disable the SPI chip select
 *
 * Input Parameters:
 *   selected: TRUE: chip selected, FALSE: chip de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(FAR struct spi_dev_s *dev, boolean selected)
{
  uint32 bit = 1 << 20;

  if (selected)
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
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   frequency: The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32 spi_setfrequency(FAR struct spi_dev_s *dev, uint32 frequency)
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
 * Name: spi_status
 *
 * Description:
 *   Get SPI/MMC status
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines
 *
 ****************************************************************************/

static ubyte spi_status(FAR struct spi_dev_s *dev)
{
  /* I don't think there is anyway to determine these things on the mcu123.com
   * board.
   */

  return SPI_STATUS_PRESENT;
}

/****************************************************************************
 * Name: spi_sndbyte
 *
 * Description:
 *   Send one byte on SPI
 *
 * Input Parameters:
 *   ch - the byte to send
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static ubyte spi_sndbyte(FAR struct spi_dev_s *dev, ubyte ch)
{
  /* Wait while the TX FIFO is full */

  while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF));

  /* Write the byte to the TX FIFO */

  putreg16(ch, LPC214X_SPI1_DR);

  /* Wait for the RX FIFO not empty */

  while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE));

  /* Get the value from the RX FIFO and return it */

  return (ubyte)getreg16(LPC214X_SPI1_DR);
}

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   buffer - A pointer to the buffer of data to be sent
 *   buflen - the length of data to send from the buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const ubyte *buffer, size_t buflen)
{
  ubyte sr;

  /* Loop while thre are bytes remaining to be sent */

  while (buflen > 0)
    {
      /* While the TX FIFO is not full and there are bytes left to send */

      while ((getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF) && buflen)
        {
          /* Send the data */

          putreg16((uint16)*buffer, LPC214X_SPI1_DR);
          buffer++;
          buflen--;
        }
    }

  /* Then discard all card responses until the TX FIFO is emptied. */

  do
    {
      /* Is there anything in the RX fifo? */

      sr = getreg8(LPC214X_SPI1_SR);
      if ((sr & LPC214X_SPI1SR_RNE) != 0)
        {
          /* Yes.. Read and discard */

          (void)getreg16(LPC214X_SPI1_DR);
        }

      /* There is a race condition where TFE may go FALSE just before
       * RNE goes true.  The nasty little delay in the following solves
       * that (it could probably be tuned to improve performance).
       */

      else if ((sr & LPC214X_SPI1SR_TFE) == 0)
        {
          up_udelay(100);
          sr = getreg8(LPC214X_SPI1_SR);
        }
    }
  while ((sr & LPC214X_SPI1SR_RNE) != 0 || (sr & LPC214X_SPI1SR_TFE) == 0);
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   buffer - A pointer to the buffer in which to recieve data
 *   buflen - the length of data that can be received in the buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR ubyte *buffer, size_t buflen)
{
  uint32 fifobytes = 0;

  /* While there is remaining to be sent (and no synchronization error has occurred) */

  while (buflen || fifobytes)
    {
      /* Fill the transmit FIFO with 0xff...
       * Write 0xff to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      while ((getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF) &&
             (fifobytes < LPC214X_SPI1_FIFOSZ) && buflen)
        {
          putreg16(0xff, LPC214X_SPI1_DR);
          buflen--;
          fifobytes++;
        }

      /* Now, read the RX data from the RX FIFO while the RX FIFO is not empty */

      while (getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE)
        {
          *buffer++ = (ubyte)getreg16(LPC214X_SPI1_DR);
          fifobytes--;
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
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  uint32 regval32;
  ubyte regval8;
  int i;

  /* Only the SPI1 interface is supported */

#ifdef CONFIG_DEBUG
  if (port != 1)
    {
      return NULL;
    }
#endif

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

  spi_setfrequency(NULL, 400000);

  /* Enable the SPI */

  regval8 = getreg8(LPC214X_SPI1_CR1);
  putreg8(regval8 | LPC214X_SPI1CR1_SSE, LPC214X_SPI1_CR1);

  for (i = 0; i < 8; i++)
    {
      (void)getreg16(LPC214X_SPI1_DR);
    }

  return &g_spidev;
}
