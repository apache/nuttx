/****************************************************************************
 * arch/avr/src/avr/up_spi.c
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

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include <avr/io.h>
#include <avr/power.h>

#include "up_internal.h"
#include "chip.h"
#include "avr.h"

#ifdef CONFIG_AVR_SPI

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct avr_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  mutex_t          lock;       /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          mode;       /* Mode 0,1,2,3 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev,
                            enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void     spi_sndblock(FAR struct spi_dev_s *dev,
                             FAR const void *buffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                              size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .lock              = spi_lock,
  .select            = avr_spiselect,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                 /* Not supported */
#endif
  .status            = avr_spistatus,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = avr_spicmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
  .registercallback  = 0,                 /* Not implemented */
};

static struct avr_spidev_s g_spidev =
{
  .spidev            =
  {
    .ops             = &g_spiops,
  },
  .lock              = NXMUTEX_INITIALIZER,
};

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
  FAR struct avr_spidev_s *priv = (FAR struct avr_spidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
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
  FAR struct avr_spidev_s *priv = (FAR struct avr_spidev_s *)dev;
  uint32_t actual;

  /* TODO: This is missing the actual logic to update the frequency.
   * The divider bits are computed but not actually used.
   */

  /* Has the request frequency changed? */

  if (frequency != priv->frequency)
    {
      /* Read the SPI status and control registers, clearing all div bits */

      uint8_t spcr = SPCR & ~((1 << SPR0) | (1 << SPR1));
      uint8_t spsr = SPSR & ~(1 << SPI2X);

      /* Select the best divider bits */

      if (frequency >= BOARD_CPU_CLOCK / 2)
        {
          spsr  |= (1 << SPI2X);
          actual = BOARD_CPU_CLOCK / 2;
        }
      else if (frequency >= BOARD_CPU_CLOCK / 4)
        {
          actual = BOARD_CPU_CLOCK / 4;
        }
      else if (frequency >= BOARD_CPU_CLOCK / 8)
        {
          spcr  |= (1 << SPR0);
          spsr  |= (1 << SPI2X);
          actual = BOARD_CPU_CLOCK / 8;
        }
      else if (frequency >= BOARD_CPU_CLOCK / 16)
        {
          spcr  |= (1 << SPR0);
          actual = BOARD_CPU_CLOCK / 16;
        }
      else if (frequency >= BOARD_CPU_CLOCK / 32)
        {
          spcr  |= (1 << SPR1);
          spsr  |= (1 << SPI2X);
          actual = BOARD_CPU_CLOCK / 32;
        }
      else if (frequency >= BOARD_CPU_CLOCK / 64)
        {
          spcr  |= (1 << SPR1);
          actual = BOARD_CPU_CLOCK / 64;
        }
      else /* if (frequency >= BOARD_CPU_CLOCK / 128) */
        {
          spcr  |= (1 << SPR0) | (1 << SPR1);
          actual = BOARD_CPU_CLOCK / 128;
        }

      /* Save the frequency setting */

      priv->frequency = frequency;
      priv->actual    = actual;
    }
  else
    {
      actual          = priv->actual;
    }

  spiinfo("Frequency %" PRId32 "->%" PRId32 "\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct avr_spidev_s *priv = (FAR struct avr_spidev_s *)dev;

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      uint8_t regval;

      /* Yes... Set SPI CR appropriately */

      regval = SPCR;
      regval &= ~((1 << CPOL) | (1 << CPHA));

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= (1 << CPHA);
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= (1 << CPOL);
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= ((1 << CPOL) | (1 << CPHA));
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      SPSR = regval;

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requested (only nbits == 8 is supported)
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  DEBUGASSERT(dev && nbits == 8);
}

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
  /* Write the data to transmitted to the SPI Data Register */

  SPDR = (uint8_t)wd;

  /* Wait for transmission to complete */

  while (!(SPSR & (1 << SPIF)));

  /* Then return the received value */

  return (uint32_t)SPDR;
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
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t nwords)
{
  FAR uint8_t *ptr = (FAR uint8_t *)buffer;

  spiinfo("nwords: %d\n", nwords);
  while (nwords-- > 0)
    {
      spi_send(dev, (uint16_t)*ptr++);
    }
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
 *            of words.  The wordsize is determined by the number of bits-
 *            per-wordselected for the SPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                          size_t nwords)
{
  FAR uint8_t *ptr = (FAR uint8_t *)buffer;

  spiinfo("nwords: %d\n", nwords);
  while (nwords-- > 0)
    {
      *ptr++ = spi_send(dev, (uint16_t)0xff);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

FAR struct spi_dev_s *avr_spibus_initialize(int port)
{
  FAR struct avr_spidev_s *priv = &g_spidev;
  irqstate_t flags;
  uint8_t regval;

  /* Make sure that clocks are provided to the SPI module */

  flags = enter_critical_section();
  power_spi_enable();

  /* Set MOSI and SCK as outputs, all others are inputs (default on reset):
   *
   * PB3: PDO/MISO/PCINT3
   * PB2: PDI/MOSI/PCINT2
   * PB1: SCK/PCINT1
   * PB0: SS/PCINT0
   */

  DDRB |= (1 << 2) | (1 << 1);

  /* - Enable SPI
   * - Set Master
   * - Set clock rate oscillator/4
   * - Set CPOL for SPI clock mode 0
   */

  SPCR = (1 << SPE) | (1 << MSTR);

  /* Set clock rate to f(osc)/8 */

  /* SPSR |= (1 << 0); */

  /* Clear status flags by reading them */

  regval = SPSR;
  UNUSED(regval);
  regval = SPDR;
  UNUSED(regval);

  /* Set the initial SPI configuration */

  priv->frequency = 0;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  leave_critical_section(flags);
  return &priv->spidev;
}
#endif /* CONFIG_AVR_SPI */
