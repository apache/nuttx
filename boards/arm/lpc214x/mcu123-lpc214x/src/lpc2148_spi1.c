/****************************************************************************
 * boards/arm/lpc214x/mcu123-lpc214x/src/lpc2148_spi1.c
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
 * Pin 27 is dedicated to a chip select, pins 30 and 31 connect to keys, and
 * pin 29 is unconnected.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc214x_power.h"
#include "lpc214x_pinsel.h"
#include "lpc214x_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking */

#define LPC214X_CCLKFREQ  (LPC214X_FOSC*LPC214X_PLL_M)
#define LPC214X_PCLKFREQ  (LPC214X_CCLKFREQ/LPC214X_APB_DIV)

/* Use either FIO or legacy GPIO */

#ifdef CONFIG_LPC214x_FIO
#  define CS_SET_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_SET_OFFSET)
#  define CS_CLR_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_CLR_OFFSET)
#  define CS_DIR_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_DIR_OFFSET)
#else
#  define CS_SET_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_SET_OFFSET)
#  define CS_CLR_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_CLR_OFFSET)
#  define CS_DIR_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_DIR_OFFSET)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency);
static uint8_t spi_status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t ch);
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t nwords);
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                          size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .lock              = spi_lock,
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .status            = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = spi_cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
  .registercallback  = 0,                 /* Not implemented */
};

static struct spi_dev_s g_spidev =
{
  &g_spiops
};

static sem_t g_exclsem = SEM_INITIALIZER(1);  /* For mutually exclusive access */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&g_exclsem);
    }
  else
    {
      ret = nxsem_post(&g_exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enable/disable the SPI slave select.   The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselecte.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  uint32_t bit = 1 << 20;

  if (selected)
    {
      /* Enable slave select (low enables) */

      spiinfo("CS asserted\n");
      putreg32(bit, CS_CLR_REGISTER);
    }
  else
    {
      /* Disable slave select (low enables) */

      spiinfo("CS de-asserted\n");
      putreg32(bit, CS_SET_REGISTER);

      /* Wait for the TX FIFO not full indication */

      while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF));
      putreg16(0xff, LPC214X_SPI1_DR);

      /* Wait until TX FIFO and TX shift buffer are empty */

      while (getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_BSY);

      /* Wait until RX FIFO is not empty */

      while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE));

      /* Then read and discard bytes until the RX FIFO is empty */

      do
        {
          getreg16(LPC214X_SPI1_DR);
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
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  uint32_t divisor = LPC214X_PCLKFREQ / frequency;

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

  spiinfo("Frequency %" PRId32 "->%" PRId32 "\n",
          frequency, (uint32_t)(LPC214X_PCLKFREQ / divisor));
  return LPC214X_PCLKFREQ / divisor;
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Get SPI/MMC status
 *
 * Input Parameters:
 *   dev -   Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines
 *
 ****************************************************************************/

static uint8_t spi_status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  /* I don't think there is anyway to determine these things on the
   * mcu123.com board.
   */

  spiinfo("Return SPI_STATUS_PRESENT\n");
  return SPI_STATUS_PRESENT;
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   Some devices require and additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#  error "spi_cmddata not implemented"
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  register uint16_t regval;

  /* Wait while the TX FIFO is full */

  while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF));

  /* Write the byte to the TX FIFO */

  putreg16((uint8_t)wd, LPC214X_SPI1_DR);

  /* Wait for the RX FIFO not empty */

  while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE));

  /* Get the value from the RX FIFO and return it */

  regval = getreg16(LPC214X_SPI1_DR);
  spiinfo("%04" PRIx32 "->%04x\n", wd, regval);
  return regval;
}

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8,
 *            the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t nwords)
{
  FAR const uint8_t *ptr = (FAR const uint8_t *)buffer;
  uint8_t sr;

  /* Loop while there are bytes remaining to be sent */

  spiinfo("nwords: %d\n", nwords);
  while (nwords > 0)
    {
      /* While the TX FIFO is not full and there are bytes left to send */

      while ((getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF) && nwords)
        {
          /* Send the data */

          putreg16((uint16_t)*ptr, LPC214X_SPI1_DR);
          ptr++;
          nwords--;
        }
    }

  /* Then discard all card responses until the RX & TX FIFOs are emptied. */

  spiinfo("discarding\n");
  do
    {
      /* Is there anything in the RX fifo? */

      sr = getreg8(LPC214X_SPI1_SR);
      if ((sr & LPC214X_SPI1SR_RNE) != 0)
        {
          /* Yes.. Read and discard */

          getreg16(LPC214X_SPI1_DR);
        }

      /* There is a race condition where TFE may go true just before
       * RNE goes true and this loop terminates prematurely.  The nasty
       * little delay in the following solves that (it could probably
       * be tuned to improve performance).
       */

      else if ((sr & LPC214X_SPI1SR_TFE) != 0)
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
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8,
 *            the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                          size_t nwords)
{
  FAR uint8_t *ptr = (FAR uint8_t *)buffer;
  uint32_t rxpending = 0;

  /* While there is remaining to be sent
   * (and no synchronization error has occurred)
   */

  spiinfo("nwords: %d\n", nwords);
  while (nwords || rxpending)
    {
      /* Fill the transmit FIFO with 0xff...
       * Write 0xff to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      spiinfo("TX: rxpending: %" PRId32 " nwords: %d\n", rxpending, nwords);
      while ((getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF) &&
             (rxpending < LPC214X_SPI1_FIFOSZ) && nwords)
        {
          putreg16(0xff, LPC214X_SPI1_DR);
          nwords--;
          rxpending++;
        }

      /* Now, read RX data from the RX FIFO while RX FIFO is not empty */

      spiinfo("RX: rxpending: %" PRId32 "\n", rxpending);
      while (getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE)
        {
          *ptr++ = (uint8_t)getreg16(LPC214X_SPI1_DR);
          rxpending--;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc214x_spibus_initialize
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

FAR struct spi_dev_s *lpc214x_spibus_initialize(int port)
{
  uint32_t regval32;
  uint8_t regval8;
  int i;

  /* Only the SPI1 interface is supported */

#ifdef CONFIG_DEBUG_FEATURES
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
   *   PINSEL1 P0.20/MAT1.3/SSEL1/EINT3  Bits 8-9=10 for P0.20
   *   (we'll control it via GPIO or FIO)
   */

  regval32  = getreg32(LPC214X_PINSEL1);
  regval32 &= ~(LPC214X_PINSEL1_P017_MASK | LPC214X_PINSEL1_P018_MASK |
                LPC214X_PINSEL1_P019_MASK | LPC214X_PINSEL1_P020_MASK);
  regval32 |= (LPC214X_PINSEL1_P017_SCK1 | LPC214X_PINSEL1_P018_MISO1 |
                LPC214X_PINSEL1_P019_MOSI1 | LPC214X_PINSEL1_P020_GPIO);
  putreg32(regval32, LPC214X_PINSEL1);

  /* Disable chip select using P0.20 (SSEL1)  (low enables) */

  regval32 = 1 << 20;
  putreg32(regval32, CS_SET_REGISTER);
  regval32 |= getreg32(CS_DIR_REGISTER);
  putreg32(regval32, CS_DIR_REGISTER);

  /* Enable peripheral clocking to SPI1 */

  regval32  = getreg32(LPC214X_PCON_PCONP);
  regval32 |= LPC214X_PCONP_PCSPI1;
  putreg32(regval32, LPC214X_PCON_PCONP);

  /* Configure 8-bit SPI mode */

  putreg16(LPC214X_SPI1CR0_DSS8BIT | LPC214X_SPI1CR0_FRFSPI,
           LPC214X_SPI1_CR0);

  /* Disable the SSP and all interrupts (we'll poll for all data) */

  putreg8(0, LPC214X_SPI1_CR1);
  putreg8(0, LPC214X_SPI1_IMSC);

  /* Set the initial clock frequency for identification mode < 400kHz */

  spi_setfrequency(NULL, 400000);

  /* Enable the SPI */

  regval8 = getreg8(LPC214X_SPI1_CR1);
  putreg8(regval8 | LPC214X_SPI1CR1_SSE, LPC214X_SPI1_CR1);

  for (i = 0; i < 8; i++)
    {
      getreg16(LPC214X_SPI1_DR);
    }

  return &g_spidev;
}
