/****************************************************************************
 * arm/arm/src/efm32/efm32_spi.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2014 Bouteville Pierre-Noel. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bouteville Pierre-Noel <pnb990@gmail.com>
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "efm32_config.h"
#include "efm32_dma.h"
#include "efm32_lowputc.h"
#include "efm32_spi.h"

#ifdef HAVE_SPI_DEVICE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ********************************************************************/
/* SPI interrupts */

#ifdef CONFIG_EFM32_SPI_DMA
#  error DMA driven SPI not yet supported
#endif

#ifdef CONFIG_EFM32_SPI_INTERRUPTS
#  error Interrupt driven SPI not yet supported
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_EFM32_SPI_INTERRUPTS) && defined(CONFIG_EFM32_SPI_DMA)
#  error Cannot enable both interrupt mode and DMA mode for SPI
#endif

/* Debug ****************************************************************************/
/* Check if SPI debug is enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_SPI
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type constant configuration one SPI peripheral */

struct efm32_spidev_s;
struct efm32_spiconfig_s
{
  uintptr_t base;        /* USART base address */
#ifdef CONFIG_EFM32_SPI_DMA
  dma_config_t rxconfig; /* RX DMA configuration (excluding transfer width) */
  dma_config_t txconfig; /* TX DMA configuration (excluding transfer width) */
#endif
  void (*select)(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                 bool selected);
  uint8_t (*status)(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
  int (*cmddata)(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
};

/* This type represents the state of one SPI peripheral */

struct efm32_spidev_s
{
  const struct spi_ops_s *spidev;         /* Externally visible part of the SPI interface */
  const struct efm32_spiconfig_s *config; /* Constant SPI hardware configuration */

#ifdef CONFIG_EFM32_SPI_DMA
  volatile int8_t rxresult;  /* Result of the RX DMA */
  volatile int8_t txresult;  /* Result of the TX DMA */
  DMA_HANDLE rxdmach;        /* RX DMA channel handle */
  DMA_HANDLE txdmach;        /* TX DMA channel handle */
  sem_t rxdmasem;            /* Wait for RX DMA to complete */
  sem_t txdmasem;            /* Wait for TX DMA to complete */
#endif

#ifndef CONFIG_SPI_OWNBUS
  sem_t exclsem;             /* Held while chip is selected for mutual exclusion */
  uint32_t frequency;        /* Requested clock frequency */
  uint32_t actual;           /* Actual clock frequency */
  uint8_t nbits;             /* Width of word in bits (8 or 16) */
  uint8_t mode;              /* Mode 0,1,2,3 */
#endif

  bool initialized; /* True: Already initialized */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low level SPI access */

static uint32_t  spi_getreg(const struct efm32_spiconfig_s *config,
                   unsigned int regoffset);
static void      spi_putreg(const struct efm32_spiconfig_s *config,
                   unsigned int regoffset, uint32_t regval);
static bool      spi_16bitmode(struct efm32_spidev_s *priv);

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int       spi_lock(struct spi_dev_s *dev, bool lock);
#endif
static void      spi_select(struct spi_dev_s *dev, enum spi_dev_e devid,
                   bool selected);
static uint32_t  spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void      spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void      spi_setbits(struct spi_dev_s *dev, int nbits);
static uint8_t   spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
static int       spi_cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                   bool cmd);
#endif
static uint16_t  spi_send(struct spi_dev_s *dev, uint16_t wd);
static void      spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                   void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void      spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                   size_t nwords);
static void      spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                   size_t nwords);
#endif

/* Initialization */

static void      spi_portinitialize(struct efm32_spidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common SPI operations */

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = spi_cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
};

#ifdef CONFIG_EFM32_USART0_ISSPI
/* Support for SPI on USART0 */

static struct efm32_spidev_s g_spi0dev;
static const struct efm32_spiconfig_s g_spi0config =
{
  .base              = EFM32_USART0_BASE,
#ifdef CONFIG_EFM32_SPI_DMA
  .rxconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART0RXDATAV) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART0) |
                       EFM32_DMA_SINGLE,
  .txconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART0TXBL) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART0)  |
                       EFM32_DMA_SINGLE,
#endif
  .select            = efm32_spi0_select,
  .status            = efm32_spi0_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = efm32_spi0_cmddata,
#endif
};
#endif /* CONFIG_EFM32_USART0_ISSPI */

#ifdef CONFIG_EFM32_USART1_ISSPI
/* Support for SPI on USART1 */

static struct efm32_spidev_s g_spi1dev;
static const struct efm32_spiconfig_s g_spi1config =
{
  .base              = EFM32_USART1_BASE,
#ifdef CONFIG_EFM32_SPI_DMA
  .rxconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART1RXDATAV) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART1) |
                       EFM32_DMA_SINGLE,
  .txconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART1TXBL) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART1)  |
                       EFM32_DMA_SINGLE,
#endif
  .select            = efm32_spi1_select,
  .status            = efm32_spi1_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = efm32_spi1_cmddata,
#endif
};
#endif /* CONFIG_EFM32_USART1_ISSPI */

#ifdef CONFIG_EFM32_USART2_ISSPI
/* Support for SPI on USART2 */

static struct efm32_spidev_s g_spi2dev;
static const struct efm32_spiconfig_s g_spi2config =
{
  .base              = EFM32_USART2_BASE,
#ifdef CONFIG_EFM32_SPI_DMA
  .rxconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART2RXDATAV) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART2) |
                       EFM32_DMA_SINGLE,
  .txconfig          = EFM32_DMA_SIGSEL(_DMA_CH_CTRL_SIGSEL_USART2TXBL) |
                       EFM32_DMA_SOURCSEL(_DMA_CH_CTRL_SOURCESEL_USART2)  |
                       EFM32_DMA_SINGLE,
#endif
  .select            = efm32_spi2_select,
  .status            = efm32_spi2_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = efm32_spi2_cmddata,
#endif
};
#endif /* CONFIG_EFM32_USART2_ISSPI */

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
 *   Read the contents of one SPI register
 *
 * Input Parameters:
 *   config    - Device-specific configuration data
 *   regoffset - Offset to the SPI register
 *
 * Returned Value:
 *   Value read from the SPI register
 *
 ****************************************************************************/

static uint32_t spi_getreg(const struct efm32_spiconfig_s *config,
                           unsigned int regoffset)
{
  uintptr_t regaddr = config->base + regoffset;
  return getreg32(regaddr);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a value to w\one SPI register
 *
 * Input Parameters:
 *   config    - Device-specific configuration data
 *   regoffset - Offset to the SPI register
 *   regval    - The value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_putreg(const struct efm32_spiconfig_s *config,
                       unsigned int regoffset, uint32_t regval)
{
  uintptr_t regaddr = config->base + regoffset;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: spi_16bitmode
 *
 * Description:
 *   Return true if the transfer is performed on 16-bit (vs 8-bit) data.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   True: 16-bit; false: 8-bit
 *
 ****************************************************************************/
/* helper SPI function */

static bool spi_16bitmode(struct efm32_spidev_s *priv)
{
#ifndef CONFIG_SPI_OWNBUS
    return (priv->nbits > 8) ? true : false;
#else
  uint32_t regval;

  regval   = spi_getreg(config, EFM32_USART_FRAME_OFFSET);
  regval  &= _USART_FRAME_DATABITS_MASK;
  regval >>= _USART_FRAME_DATABITS_SHIFT;
  return (regval > _USART_FRAME_DATABITS_EIGHT);
#endif
}

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
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

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      while (sem_wait(&priv->exclsem) != 0)
        {
          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)sem_post(&priv->exclsem);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enable/disable the SPI chip select. 
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

static void spi_select(struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool selected)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;
  DEBUGASSERT(config->select);

  /* Defer to the board chip select logic */

  config->select(dev, devid, selected);
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

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;
  uint32_t clkdiv;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Has the frequency changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (frequency != priv->frequency)
    {
#endif
      /* We want to use integer division to avoid forcing in float division
       * utils, and yet keep rounding effect errors to a minimum.
       *
       * CLKDIV in synchronous mode is given by:
       *
       * CLKDIV = 256 * (fHFPERCLK/(2 * br) - 1)
       * or
       * CLKDIV = (256 * fHFPERCLK)/(2 * br) - 256 = (128 * fHFPERCLK)/br - 256
       *
       * The basic problem with integer division in the above formula is that
       * the dividend (128 * fHFPERCLK) may become higher than max 32 bit
       * integer. Yet, we want to evaluate dividend first before dividing in
       * order to get as small rounding effects as possible. We do not want
       * to make too harsh restrictions on max fHFPERCLK value either.
       *
       * One can possibly factorize 128 and br. However, since the last
       * 6 bits of CLKDIV are don't care, we can base our integer arithmetic
       * on the below formula without loosing any extra precision:
       *
       * CLKDIV / 64 = (2 * fHFPERCLK)/br - 4
       *
       * and calculate 1/64 of CLKDIV first. This allows for fHFPERCLK
       * up to 2GHz without overflowing a 32 bit value!
       */

      /* Calculate and set CLKDIV with fractional bits */

      clkdiv  = (2 * BOARD_HFPERCLK_FREQUENCY + (frequency - 1)) / frequency;
      clkdiv  = 64 * (clkdiv - 4);

      /* Make sure we don't use fractional bits by rounding CLKDIV up (and
       * thus reducing baudrate, not increasing baudrate above.
       * specified value).
       */

      clkdiv = (clkdiv + 0xc0) & 0xffffff00;

      /* Verify that resulting clock divider is within limits */

      DEBUGASSERT(clkdiv <= _USART_CLKDIV_MASK);

      clkdiv &= _USART_CLKDIV_DIV_MASK;
      spi_putreg(config, EFM32_USART_CLKDIV_OFFSET, clkdiv);

      spivdbg("Frequency %d\n", frequency);

#ifndef CONFIG_SPI_OWNBUS
      /* Save the frequency selection so that subsequent reconfigurations
       * will be faster.
       */

      priv->frequency = frequency;
    }
#endif

  return frequency;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns void
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;
  uint32_t setting;
  uint32_t regval;

  spivdbg("mode=%d\n", mode);

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Has the mode changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (mode != priv->mode)
    {
#endif
      setting = 0;
      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          setting = USART_CTRL_CLKPOL_IDLELOW | USART_CTRL_CLKPHA_SAMPLELEADING;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          setting = USART_CTRL_CLKPOL_IDLELOW | USART_CTRL_CLKPHA_SAMPLETRAILING;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          setting = USART_CTRL_CLKPOL_IDLEHIGH | USART_CTRL_CLKPHA_SAMPLELEADING;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          setting = USART_CTRL_CLKPOL_IDLEHIGH | USART_CTRL_CLKPHA_SAMPLETRAILING;
          break;

        default:
          return;
        }

      regval  = spi_getreg(config, EFM32_USART_CTRL_OFFSET);
      regval &= ~(_USART_CTRL_CLKPOL_MASK | _USART_CTRL_CLKPHA_MASK);
      regval |= setting;
      spi_putreg(config, EFM32_USART_CLKDIV_OFFSET, regval);

#ifndef CONFIG_SPI_OWNBUS
      /* Save the mode so that subsequent re-configurations will be faster */

        priv->mode = mode;
    }
#endif
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;
  uint32_t regval;
  uint32_t setting;
  unsigned int truebits;

  spivdbg("nbits=%d\n", nbits);

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Has the number of bits changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (nbits != priv->nbits)
    {
#endif
      regval = spi_getreg(config, EFM32_USART_CTRL_OFFSET);
      if (nbits < 0)
        {
          regval &= ~USART_CTRL_MSBF;
          truebits = -nbits;
        }
      else
        {
          regval |= USART_CTRL_MSBF;
          truebits = nbits;
        }

      spi_putreg(config, EFM32_USART_CLKDIV_OFFSET, regval);

      switch (truebits)
        {
        case  4:
          setting = USART_FRAME_DATABITS_FOUR;
          break;

        case  5:
          setting = USART_FRAME_DATABITS_FIVE;
          break;

        case  6:
          setting = USART_FRAME_DATABITS_SIX;
          break;

        case  7:
          setting = USART_FRAME_DATABITS_SEVEN;
          break;

        case  8:
          setting = USART_FRAME_DATABITS_EIGHT;
          break;

        case  9:
          setting = USART_FRAME_DATABITS_NINE;
          break;

        case  10:
          setting = USART_FRAME_DATABITS_TEN;
          break;

        case  11:
          setting = USART_FRAME_DATABITS_ELEVEN;
          break;

        case  12:
          setting = USART_FRAME_DATABITS_TWELVE;
          break;

        case  13:
          setting = USART_FRAME_DATABITS_THIRTEEN;
          break;

        case  14:
          setting = USART_FRAME_DATABITS_FOURTEEN;
          break;

        case  15:
          setting = USART_FRAME_DATABITS_FIFTEEN;
          break;

        case  16:
          setting = USART_FRAME_DATABITS_SIXTEEN;
          break;

        default:
          return;
        }

      regval  = spi_getreg(config, EFM32_USART_FRAME_OFFSET);
      regval &= _USART_FRAME_DATABITS_MASK;
      regval |= setting;
      spi_putreg(config, EFM32_USART_FRAME_OFFSET, regval);

#ifndef CONFIG_SPI_OWNBUS
      /* Save the selection so the subsequence re-configurations will be faster */

      priv->nbits = nbits;
    }
#endif
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Get SPI/MMC status.  Optional.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/

static uint8_t spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Defer to the board chip status logic */

  return config->status(dev, devid);
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
static int spi_cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                       bool cmd);
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;
  DEBUGASSERT(config->cmddata);

  /* Defer to the board chip cmd/data logic */

  return config->cmddata(dev, devid, cmd);
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

static uint16_t spi_send(struct spi_dev_s *dev, uint16_t wd)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;
  uint16_t ret;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  /* Wait until there is space in the TX buffer */

  while ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_TXBL) == 0);

  /* Write the data */

  spi_putreg(config, EFM32_USART_TXDATA_OFFSET, (uint32_t)wd);

  /* Wait for receive data to be available */

  while ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_RXDATAV) == 0);
  ret = (uint16_t)spi_getreg(config, EFM32_USART_RXDATA_OFFSET);

  spivdbg("Sent: %04x Return: %04x \n", wd, ret);
  return ret;
}

/****************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if !defined(CONFIG_EFM32_SPI_DMA)
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
#else
static void spi_exchange_nodma(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords)
#endif
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;
  size_t unrecvd;
  size_t unsent;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

  spivdbg("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (spi_16bitmode(priv))
    {
      /* 16-bit mode */

      const uint16_t *src  = (const uint16_t*)txbuffer;;
            uint16_t *dest = (uint16_t*)rxbuffer;
            uint16_t  word;

      unrecvd = nwords;
      unsent  = nwords;
      while (unrecvd > 0)
        {
            /* Send data while there is space in the TX buffer.  This should
             * provide some benefit when the depth of the TC buffer is > 1
             */

           while ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_TXBL) != 0 &&
                   unsent > 0)
             {
              /* Get the next word to write.  Is there a source buffer? */

              if (src)
                {
                  word = *src++;
                }
              else
                {
                  word = 0xffff;
                }

              /* Send one word */

              spi_putreg(config, EFM32_USART_TXDATA_OFFSET, (uint32_t)word);
              unsent--;
            }

          /* Receive data where there is data available */

          while ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_RXDATAV) != 0 &&
                 unrecvd > 0)
            {
              /* Receive the data */

              word = (uint16_t)spi_getreg(config, EFM32_USART_RXDATA_OFFSET);
              unrecvd--;

              /* Is there a buffer to receive the return value? */

              if (dest)
                {
                  *dest++ = word;
                }
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src  = (const uint8_t*)txbuffer;;
            uint8_t *dest = (uint8_t*)rxbuffer;
            uint8_t  word;

      unrecvd = nwords;
      unsent  = nwords;
      while (unrecvd > 0)
        {
            /* Send data while there is space in the TX buffer.  This should
             * provide some benefit when the depth of the TC buffer is > 1
             */

           while ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_TXBL) != 0 &&
                   unsent > 0)
             {
              /* Get the next word to write.  Is there a source buffer? */

              if (src)
                {
                  word = *src++;
                }
              else
                {
                  word = 0xff;
                }

              /* Send one word */

              spi_putreg(config, EFM32_USART_TXDATA_OFFSET, (uint32_t)word);
              unsent--;
            }

          /* Receive data where there is data available */

          while ((spi_getreg(config, EFM32_USART_STATUS_OFFSET) & USART_STATUS_RXDATAV) != 0 &&
                 unrecvd > 0)
            {
              /* Receive the data */

              word = (uint8_t)spi_getreg(config, EFM32_USART_RXDATA_OFFSET);
              unrecvd--;

              /* Is there a buffer to receive the return value? */

              if (dest)
                {
                  *dest++ = word;
                }
            }
        }
    }
 
  DEBUGASSERT(unsent == 0);
}

/*************************************************************************
 * Name: spi_exchange (with DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_SPI_DMA
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct efm32_spidev_s *priv = (struct efm32_spidev_s *)dev;
  const struct efm32_spiconfig_s *config;

  DEBUGASSERT(priv && priv->config);
  config = priv->config;

#ifdef CONFIG_STM32_DMACAPABLE
  if ((txbuffer && !stm32_dmacapable((uint32_t)txbuffer, nwords, priv->txccr)) ||
      (rxbuffer && !stm32_dmacapable((uint32_t)rxbuffer, nwords, priv->rxccr)))
    {
      /* Unsupported memory region, fall back to non-DMA method. */

      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
    }
  else
#endif
    {
      static uint16_t rxdummy = 0xffff;
      static const uint16_t txdummy = 0xffff;

      spivdbg("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

      /* Setup DMAs */

      spi_dmarxsetup(priv, rxbuffer, &rxdummy, nwords);
      spi_dmatxsetup(priv, txbuffer, &txdummy, nwords);

      /* Start the DMAs */

      spi_dmarxstart(priv);
      spi_dmatxstart(priv);

      /* Then wait for each to complete */

      spi_dmarxwait(priv);
      spi_dmatxwait(priv);
    }
}
#endif /* CONFIG_EFM32_SPI_DMA */

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *txbuffer, size_t nwords)
{
  spivdbg("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(struct spi_dev_s *dev, void *rxbuffer, size_t nwords)
{
  spivdbg("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_portinitialize
 *
 * Description:
 *      Initialize the selected SPI port in its default state 
 *  (Master, 8-bit, mode 0, etc.)
 *
 * Input Parameter:
 *      priv   - private SPI device structure
 *
 * Returned Value:
 *      None
 *
 ****************************************************************************/

static void spi_portinitialize(struct efm32_spidev_s *priv)
{
  const struct efm32_spiconfig_s *config = priv->config;
  uint32_t regval;

  /* Initialize USART registers to HW reset state. */

  efm32_uart_reset(config->base);

  /* Configure CR1. Default configuration:
   *   Mode 0:                          USART_CTRL_CLKPOL_IDLELOW 
   *                                    USART_CTRL_CLKPHA_SAMPLELEADING  
   *   8-bit:                           USART_FRAME_DATABITS_EIGHT 
   *   MSB tranmitted first:            ~USART_CTRL_MSBF
   */

  /* Set bits for synchronous mode */

  regval = _USART_CTRL_RESETVALUE | USART_CTRL_SYNC | USART_CTRL_CLKPOL_IDLELOW |
            USART_CTRL_CLKPHA_SAMPLELEADING;
  spi_putreg(config, EFM32_USART_CTRL_OFFSET, regval);

  /* LSB First */

  regval &= ~USART_CTRL_MSBF;
  spi_putreg(config, EFM32_USART_CTRL_OFFSET, regval);

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;
#endif

  /* 8 bits */

  regval = USART_FRAME_DATABITS_EIGHT | USART_FRAME_STOPBITS_DEFAULT |
           USART_FRAME_PARITY_DEFAULT;
  spi_putreg(config, EFM32_USART_CTRL_OFFSET, regval);

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Master mode */

  spi_putreg(config, EFM32_USART_CMD_OFFSET, USART_CMD_MASTEREN);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

#ifndef CONFIG_SPI_OWNBUS
  sem_init(&priv->exclsem, 0, 1);
#endif

  /* Initialize the SPI semaphores that is used to wait for DMA completion */

#ifdef CONFIG_EFM32_SPI_DMA
  spi_portinitialize_dma(priv);
#endif

  /* Enable SPI */

  spi_putreg(config, EFM32_USART_CMD_OFFSET, USART_CMD_RXEN | USART_CMD_TXEN);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_spi_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   port - SPI port number to initialize.  One of {0,1,2}
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *efm32_spi_initialize(int port)
{
  const struct efm32_spiconfig_s *config;
  struct efm32_spidev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_EFM32_USART0_ISSPI
  if (port == 0)
    {
      priv   = &g_spi0dev;
      config = &g_spi0config;
    }
  else
#endif
#ifdef CONFIG_EFM32_USART1_ISSPI
  if (port == 1)
    {
      priv   = &g_spi1dev;
      config = &g_spi1config;
    }
  else
#endif
#ifdef CONFIG_EFM32_USART2_ISSPI
  if (port == 2)
    {
      priv   = &g_spi2dev;
      config = &g_spi2config;
    }
  else
#endif
    {
      spidbg("ERROR: Unsupported SPI port: %d\n", port);
      return NULL;
    }

  /* Has this port been initialized yet? */

  if (!priv->initialized)
    {
      /* No, then initialize it now */

      flags = irqsave();

      /* Initialize the state structure */

      priv->spidev    = &g_spiops;
      priv->config     = config; 

#ifdef CONFIG_EFM32_SPI_DMA
      /* Allocate two DMA channels... one for the RX and one for the TX side
       * of the transfer.
       */

      priv->rxdmach = efm3_dmachannel();
      if (!priv->rxdmach)
        {
          spidbg("ERROR: Failed to allocate the RX DMA channel for SPI port: %d\n", port);
          return NULL;
        }

      priv-txdmach = efm3_dmachannel();
      if (!priv->txdmach)
        {
          spidbg("ERROR: Failed to allocate the TX DMA channel for SPI port: %d\n", port);
          efm32_dmafree(priv->rxdmach);
          priv->rxdmach = NULL;
          return NULL;
        }

      /* Initialized semaphores used to wait for DMA completion */

      sem_init(&priv->rxdmasem, 0, 0);
      sem_init(&priv->txdmasem, 0, 0);
#endif

      /* NOTES:
       *
       * 1. USART GPIO pins were configured in efm32_lowsetup().  Chip
       *    select pins must be configured by board specific logic before
       *    efm32_spi_initialize() is called.
       * 2. Clocking for the USART as also enabled in up_lowsetup();
       */

        spi_portinitialize(priv);

       /* Now we are initialized */

       priv->initialized = true;
       irqrestore(flags);
    }

   return (struct spi_dev_s *)priv;
}

#endif /* HAVE_SPI_DEVICE */
