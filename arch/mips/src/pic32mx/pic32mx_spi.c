/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_spi.c
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

#include "mips_internal.h"
#include "chip.h"
#include "pic32mx.h"
#include "pic32mx_spi.h"

#if defined(CONFIG_PIC32MX_SPI1) || defined(CONFIG_PIC32MX_SPI2) || \
    defined(CONFIG_PIC32MX_SPI3) || defined(CONFIG_PIC32MX_SPI4)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#ifdef CONFIG_SPI_EXCHANGE
  /* See arch/mips/src/pic32mz/pic32mz_spi.c for an implementation */

#  error CONFIG_SPI_EXCHANGE not supported by this driver
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the SSP driver */

struct pic32mx_dev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         base;       /* SPI register base address */
#ifdef CONFIG_PIC32MX_SPI_INTERRUPTS
  uint8_t          vector;     /* Interrupt vector number(for attaching) */
  uint8_t          eirq;       /* SPI fault interrupt number */
  uint8_t          rxirq;      /* SPI receive done interrupt number */
  uint8_t          txirq;      /* SPI transfer done interrupt number */
#endif
  mutex_t          lock;       /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level register access */

static uint32_t spi_getreg(struct pic32mx_dev_s *priv,
                  unsigned int offset);
static void     spi_putreg(struct pic32mx_dev_s *priv,
                  unsigned int offset, uint32_t value);

/* SPI methods */

static int      spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency);
static void     spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void     spi_sndblock(struct spi_dev_s *dev,
                             const void *buffer, size_t nwords);
static void     spi_recvblock(struct spi_dev_s *dev, void *buffer,
                              size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PIC32MX_SPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
  .select            = pic32mx_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                    /* Not supported */
#endif
  .status            = pic32mx_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mx_spi1cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mx_spi1register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static struct pic32mx_dev_s g_spi1dev =
{
  .spidev            =
  {
    &g_spi1ops
  },
  .base              = PIC32MX_SPI1_K1BASE,
#ifdef CONFIG_PIC32MX_SPI_INTERRUPTS
  .vector            = PIC32MX_IRQ_SPI1,
  .eirq              = PIC32MX_IRQSRC_SPI1E,
  .rxirq             = PIC32MX_IRQSRC_SPI1RX,
  .txirq             = PIC32MX_IRQSRC_SPI1TX,
#endif
};
#endif

#ifdef CONFIG_PIC32MX_SPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock              = spi_lock,
  .select            = pic32mx_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mx_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mx_spi2cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mx_spi2register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static struct pic32mx_dev_s g_spi2dev =
{
  .spidev            =
  {
    &g_spi2ops
  },
  .base              = PIC32MX_SPI2_K1BASE,
#ifdef CONFIG_PIC32MX_SPI_INTERRUPTS
  .vector            = PIC32MX_IRQ_SPI2,
  .eirq              = PIC32MX_IRQSRC_SPI2E,
  .rxirq             = PIC32MX_IRQSRC_SPI2RX,
  .txirq             = PIC32MX_IRQSRC_SPI2TX,
#endif
};
#endif

#ifdef CONFIG_PIC32MX_SPI3
static const struct spi_ops_s g_spi3ops =
{
  .lock              = spi_lock,
  .select            = pic32mx_spi3select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mx_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mx_spi3cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mx_spi3register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static struct pic32mx_dev_s g_spi3dev =
{
  .spidev            =
  {
    &g_spi3ops
  },
  .base              = PIC32MX_SPI3_K1BASE,
#ifdef CONFIG_PIC32MX_SPI_INTERRUPTS
  .vector            = PIC32MX_IRQ_SPI4,
  .eirq              = PIC32MX_IRQSRC_SPI3E,
  .rxirq             = PIC32MX_IRQSRC_SPI3RX,
  .txirq             = PIC32MX_IRQSRC_SPI3TX,
#endif
};
#endif

#ifdef CONFIG_PIC32MX_SPI4
static const struct spi_ops_s g_spi4ops =
{
  .lock              = spi_lock,
  .select            = pic32mx_spi4select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = pic32mx_spi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = pic32mx_spi4cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = pic32mx_spi4register, /* Provided externally */
#else
  .registercallback  = 0,                    /* Not implemented */
#endif
};

static struct pic32mx_dev_s g_spi4dev =
{
  .spidev            =
  {
    &g_spi4ops
  },
  .base              = PIC32MX_SPI4_K1BASE,
#ifdef CONFIG_PIC32MX_SPI_INTERRUPTS
  .vector            = PIC32MX_IRQ_SPI4,
  .eirq              = PIC32MX_IRQSRC_SPI4E,
  .rxirq             = PIC32MX_IRQSRC_SPI4RX,
  .txirq             = PIC32MX_IRQSRC_SPI4TX,
#endif
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Return the contents of one, 32-bit SPI register
 *
 * Input Parameters:
 *   priv   - A pointer to a PIC32MX SPI state structure
 *   offset - Offset from the SPI base address to the register of interest
 *
 * Returned Value:
 *   The current contents of the register
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MX_SPI_REGDEBUG
static uint32_t spi_getreg(struct pic32mx_dev_s *priv,
                           unsigned int offset)
{
  /* Last address, value, and count */

  static uint32_t prevaddr = 0;
  static uint32_t prevalue = 0;
  static uint32_t count = 0;

  /* New address and value */

  uint32_t addr;
  uint32_t value;

  /* Read the value from the register */

  addr  = priv->base + offset;
  value = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && value == prevalue)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              _info("...\n");
            }

          return value;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          _info("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      prevalue = value;
      count    = 1;
    }

  /* Show the register value read */

  _info("%08x->%08x\n", addr, value);
  return value;
}
#else
static uint32_t spi_getreg(struct pic32mx_dev_s *priv,
                           unsigned int offset)
{
  return getreg32(priv->base + offset);
}
#endif

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a value to one, 32-bit SPI register
 *
 * Input Parameters:
 *   priv   - A pointer to a PIC32MX SPI state structure
 *   offset - Offset from the SPI base address to the register of interest
 *   value  - The value to write to the SPI register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MX_SPI_REGDEBUG
static void spi_putreg(struct pic32mx_dev_s *priv, unsigned int offset,
                       uint32_t value)
{
  uint32_t addr;

  /* Get the address to write to */

  addr = priv->base + offset;

  /* Show the register value being written */

  _info("%08x<-%08x\n", addr, value);

  /* Then do the write */

  putreg32(value, addr);
}
#else
static void spi_putreg(struct pic32mx_dev_s *priv, unsigned int offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}
#endif

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

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct pic32mx_dev_s *priv = (struct pic32mx_dev_s *)dev;
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

static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  struct pic32mx_dev_s *priv = (struct pic32mx_dev_s *)dev;
  uint32_t divisor;
  uint32_t actual;
  uint32_t regval;

  spiinfo("Old frequency: %" PRId32 " actual: %" PRId32
          " New frequency: %" PRId32 "\n",
          priv->frequency, priv->actual, frequency);

  /* Check if the requested frequency is the same as the frequency
   * selection
   */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* Calculate the divisor
   *
   * frequency = BOARD_PBCLOCK / (2 * divisor), or
   * divisor  = (BOARD_PBCLOCK / 2) / frequency
   */

  divisor = (BOARD_PBCLOCK / 2) / frequency;

  /* The a BRG register value is that divisor minus one
   *
   * frequency = BOARD_PBCLOCK /(2 * (BRG + 1)), or
   * BRG       = (BOARD_PBCLOCK / 2) / frequency - 1
   */

  regval = divisor;
  if (regval > 0)
    {
      regval--;
    }

  /* Save the new BRG value */

  spi_putreg(priv, PIC32MX_SPI_BRG_OFFSET, regval);
  spiinfo("PBCLOCK: %d frequency: %" PRId32 " divisor: %" PRId32
          " BRG: %" PRId32 "\n",
          BOARD_PBCLOCK, frequency, divisor, regval);

  /* Calculate the new actual frequency.
   *
   * frequency = BOARD_PBCLOCK / (2 * divisor)
   */

  actual = (BOARD_PBCLOCK / 2) / divisor;

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("New frequency: %" PRId32 " Actual: %" PRId32 "\n",
          frequency, actual);
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

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct pic32mx_dev_s *priv = (struct pic32mx_dev_s *)dev;
  uint32_t regval;

  spiinfo("Old mode: %d New mode: %d\n", priv->mode, mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CON register appropriately.
       *
       * Standard terminology is as follows:
       *
       *   Mode  CPOL  CPHA
       *     0     0     0
       *     1     0     1
       *     2     1     0
       *     3     1     1
       *
       *   CPOL=0: The inactive value of the clock is zero
       *    CPOL=1: The inactive value of the clock is one
       *   CPHA=0: Data is captured on the clock's inactive-to-active edge
       *           and data is propagated on a active-to-inactive edge.
       *   CPHA=1: Data is captured on the clock's active-to-inactive edge
       *           and data is propagated on a active-to-inactive edge.
       *
       * CON Register mapping:
       *   CPOL=0 corresponds to CON:CKP=0; CPOL=1 corresponds to CON:CKP=1
       *   CPHA=0 corresponds to CON:CKE=1; CPHA=1 corresponds to CON:CKE=1
       *
       * In addition, the CON register supports SMP: SPI Data Input Sample
       * Phase bit:
       *
       *    1 = Input data sampled at end of data output time
       *    0 = Input data sampled at middle of data output time
       *
       * Which is hardcoded to 1.
       */

      regval = spi_getreg(priv, PIC32MX_SPI_CON_OFFSET);
      regval &= ~(SPI_CON_CKP | SPI_CON_CKE);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= SPI_CON_CKE;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= SPI_CON_CKP;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= (SPI_CON_CKP | SPI_CON_CKE);
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spi_putreg(priv, PIC32MX_SPI_CON_OFFSET, regval);
      spiinfo("CON: %08" PRIx32 "\n", regval);

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
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct pic32mx_dev_s *priv = (struct pic32mx_dev_s *)dev;
  uint32_t setting;
  uint32_t regval;

  DEBUGASSERT(priv != NULL);
  spiinfo("Old nbits: %d New nbits: %d\n", priv->nbits, nbits);
  DEBUGASSERT(nbits > 7 && nbits < 17);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Yes... Set the CON register appropriately */

      if (nbits == 8)
        {
          setting = SPI_CON_MODE_8BIT;
        }
      else if (nbits == 16)
        {
          setting = SPI_CON_MODE_8BIT;
        }
      else if (nbits == 32)
        {
          setting = SPI_CON_MODE_8BIT;
        }
      else
        {
          spierr("ERROR: Unsupported nbits: %d\n", nbits);
          return;
        }

      regval = spi_getreg(priv, PIC32MX_SPI_CON_OFFSET);
      regval &= ~SPI_CON_MODE_MASK;
      regval |= setting;
      regval = spi_getreg(priv, PIC32MX_SPI_CON_OFFSET);
      spiinfo("CON: %08" PRIx32 "\n", regval);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      priv->nbits = nbits;
    }
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

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct pic32mx_dev_s *priv = (struct pic32mx_dev_s *)dev;

  spiinfo("wd: %04" PRIx32 "\n", wd);

  /* Write the data to transmitted to the SPI Data Register */

  spi_putreg(priv, PIC32MX_SPI_BUF_OFFSET, wd);

#ifdef CONFIG_PIC32MX_SPI_ENHBUF
  /* Wait for the SPIRBE bit in the SPI Status Register to be set to 0. In
   * enhanced buffer mode, the SPIRBE bit will be cleared in  when the
   * receive buffer is not empty.
   */

  while ((spi_getreg(priv, PIC32MX_SPI_STAT_OFFSET) & SPI_STAT_SPIRBE) != 0);

#else
  /* Wait for the SPIRBF bit in the SPI Status Register to be set to 1. In
   * normal mode, the SPIRBF bit will be set when receive data is available.
   */

  while ((spi_getreg(priv, PIC32MX_SPI_STAT_OFFSET) & SPI_STAT_SPIRBF) == 0);
#endif

  /* Return the SPI data */

  return spi_getreg(priv, PIC32MX_SPI_BUF_OFFSET);
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

static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords)
{
  struct pic32mx_dev_s *priv = (struct pic32mx_dev_s *)dev;
  uint8_t *ptr = (uint8_t *)buffer;
  uint32_t regval;
  uint8_t data;

  spiinfo("nwords: %d\n", nwords);
  while (nwords)
    {
      /* Write the data to transmitted to the SPI Data Register */

      data = *ptr++;
      spi_putreg(priv, PIC32MX_SPI_BUF_OFFSET, (uint32_t)data);

#ifdef CONFIG_PIC32MX_SPI_ENHBUF
      /* Wait for the SPIRBE bit in the SPI Status Register to be set to 0.
       * In enhanced buffer mode, the SPIRBE bit will be cleared in when
       * the receive buffer is not empty.
       */

      while ((spi_getreg(priv, PIC32MX_SPI_STAT_OFFSET) &
              SPI_STAT_SPIRBE) != 0);
#else
      /* Wait for the SPIRBF bit in the SPI Status Register to be set to 1.
       * In normal mode, the SPIRBF bit will be set when receive data is
       * available.
       */

      while ((spi_getreg(priv, PIC32MX_SPI_STAT_OFFSET) &
              SPI_STAT_SPIRBF) == 0);
#endif

      /* Read from the buffer register to clear the status bit */

      regval = spi_getreg(priv, PIC32MX_SPI_BUF_OFFSET);
      UNUSED(regval);
      nwords--;
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
 *            of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits > 8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(struct spi_dev_s *dev, void *buffer,
                          size_t nwords)
{
  struct pic32mx_dev_s *priv = (struct pic32mx_dev_s *)dev;
  uint8_t *ptr = (uint8_t *)buffer;

  spiinfo("nwords: %d\n", nwords);
  while (nwords)
    {
      /* Write some dummy data to the SPI Data Register in order to clock the
       * read data.
       */

      spi_putreg(priv, PIC32MX_SPI_BUF_OFFSET, 0xff);

#ifdef CONFIG_PIC32MX_SPI_ENHBUF
      /* Wait for the SPIRBE bit in the SPI Status Register to be set to 0.
       * In enhanced buffer mode, the SPIRBE bit will be cleared in when
       * the receive buffer is not empty.
       */

      while ((spi_getreg(priv, PIC32MX_SPI_STAT_OFFSET) &
              SPI_STAT_SPIRBE) != 0);
#else
      /* Wait for the SPIRBF bit in the SPI Status Register to be set to 1.
       * In normal mode, the SPIRBF bit will be set when receive data is
       * available.
       */

      while ((spi_getreg(priv, PIC32MX_SPI_STAT_OFFSET) &
              SPI_STAT_SPIRBF) == 0);
#endif

      /* Read the received data from the SPI Data Register */

      *ptr++ = (uint8_t)spi_getreg(priv, PIC32MX_SPI_BUF_OFFSET);
      nwords--;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_spibus_initialize
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

struct spi_dev_s *pic32mx_spibus_initialize(int port)
{
  struct pic32mx_dev_s *priv;
  irqstate_t flags;
  uint32_t regval;

  spiinfo("port: %d\n", port);

  /* Select the SPI state structure for this port */

#ifdef CONFIG_PIC32MX_SPI1
  if (port == 1)
    {
      priv = &g_spi1dev;
    }
  else
#endif
#ifdef CONFIG_PIC32MX_SPI2
  if (port == 2)
    {
      priv = &g_spi2dev;
    }
  else
#endif
#ifdef CONFIG_PIC32MX_SPI3
  if (port == 3)
    {
      priv = &g_spi3dev;
    }
  else
#endif
#ifdef CONFIG_PIC32MX_SPI4
  if (port == 4)
    {
      priv = &g_spi4dev;
    }
  else
#endif
    {
      spierr("ERROR: Unsupported port: %d\n", port);
      return NULL;
    }

  /* Disable SPI interrupts */

  flags = enter_critical_section();
#ifdef CONFIG_PIC32MX_SPI_INTERRUPTS
  up_disable_irq(priv->eirq);
  up_disable_irq(priv->txirq);
  up_disable_irq(priv->rxirq);
#endif

  /* Stop and reset the SPI module by clearing the ON bit in the CON
   * register.
   */

  spi_putreg(priv, PIC32MX_SPI_CON_OFFSET, 0);

  /* Clear the receive buffer by reading from the BUF register */

  regval = spi_getreg(priv, PIC32MX_SPI_BUF_OFFSET);

#ifdef CONFIG_PIC32MX_SPI_INTERRUPTS
  /* Attach the interrupt vector.  We do this early to make sure that the
   * resource is available.
   */

  ret = irq_attach(priv->vector, spi_interrupt, NULL);
  if (ret < 0)
    {
      spierr("ERROR: Failed to attach vector: %d port: %d\n",
             priv->vector, port);
      goto errout;
    }
#endif

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Clear the SPIROV overflow bit (SPIxSTAT:6). */

  spi_putreg(priv, PIC32MX_SPI_STATCLR_OFFSET, SPI_STAT_SPIROV);

  /* Initial settings 8 bit +  master mode + mode 0.  NOTE that MSSEN
   * not set:  The slave select pin must be driven manually via the
   * board-specific pic32mx_spiNselect() interface.
   */

  regval = (SPI_CON_MSTEN | SPI_CON_SMP | SPI_CON_MODE_8BIT | SPI_CON_ON);

  /* Set the ENHBUF bit if using Enhanced Buffer mode. */

#ifdef CONFIG_PIC32MX_SPI_ENHBUF
  regval |= (SPI_CON_ENHBUF | SPI_CON_RTXISEL_HALF | SPI_CON_STXISEL_HALF);
#endif
  spi_putreg(priv, PIC32MX_SPI_CON_OFFSET, regval);
  spiinfo("CON: %08" PRIx32 "\n", regval);

  /* Set the initial SPI configuration */

  priv->nbits = 8;
  priv->mode  = SPIDEV_MODE0;

  /* Initialize the SPI mutex that enforces mutually exclusive access */

  nxmutex_init(&priv->lock);

#ifdef CONFIG_PIC32MX_SPI_INTERRUPTS
  /* Enable interrupts at the SPI controller */

  up_enable_irq(priv->eirq);
  up_enable_irq(priv->txirq);
  up_enable_irq(priv->rxirq);

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set the SPI interrupt priority */

  ret = up_prioritize_irq(priv->vector, CONFIG_PIC32MX_SPI_PRIORITY)
  if (ret < 0)
    {
      spierr("ERROR: up_prioritize_irq failed: %d\n", ret);
      goto errout;
    }
#endif
#endif

  /* Enable interrupts at the interrupt controller */

  leave_critical_section(flags);
  return &priv->spidev;

#ifdef CONFIG_PIC32MX_SPI_INTERRUPTS
errout:
  leave_critical_section(flags);
  return NULL;
#endif
}

#endif /* CONFIG_PIC32MX_SPI */
