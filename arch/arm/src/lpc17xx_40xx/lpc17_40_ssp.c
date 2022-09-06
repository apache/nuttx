/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_ssp.c
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

#include "arm_internal.h"
#include "chip.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_gpio.h"
#include "lpc17_40_ssp.h"

#if defined(CONFIG_LPC17_40_SSP0) || defined(CONFIG_LPC17_40_SSP1) || \
    defined(CONFIG_LPC17_40_SSP2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* This driver does not support the SPI exchange method. */

#ifdef CONFIG_SPI_EXCHANGE
#  error "CONFIG_SPI_EXCHANGE must not be defined in the configuration"
#endif

/* SSP Clocking *************************************************************/

#if defined(LPC176x)
/* The CPU clock by 1, 2, 4, or 8 to get the SSP peripheral clock
 * (SSP_CLOCK).  SSP_CLOCK may be further divided by 2-254 to get the SSP
 * clock.  If we want a usable range of 4KHz to 25MHz for the SSP, then:
 *
 * 1. SSPCLK must be greater than (2*25MHz) = 50MHz, and
 * 2. SSPCLK must be less than (254*40Khz) = 101.6MHz.
 *
 * If we assume that CCLK less than or equal to 100MHz, we can just
 * use the CCLK undivided to get the SSP_CLOCK.
 */

#  if LPC17_40_CCLK > 100000000
#    error "CCLK <= 100,000,000 assumed"
#  endif

#  define SSP_PCLKSET_DIV    SYSCON_PCLKSEL_CCLK
#  define SSP_CLOCK          LPC17_40_CCLK

#elif defined(LPC178x_40xx)
/* All peripherals are clocked by the same peripheral clock in the
 * LPC178x/40xx family.
 */

#  define SSP_CLOCK          BOARD_PCLK_FREQUENCY

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the SSP driver */

struct lpc17_40_sspdev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         sspbase;    /* SPIn base address */
#ifdef CONFIG_LPC17_40_SSP_INTERRUPTS
  uint8_t          sspirq;     /* SPI IRQ number */
#endif
  mutex_t          lock;       /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (4 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t ssp_getreg(struct lpc17_40_sspdev_s *priv,
                                  uint8_t offset);
static inline void ssp_putreg(struct lpc17_40_sspdev_s *priv,
                              uint8_t offset, uint32_t value);

/* SPI methods */

static int      ssp_lock(struct spi_dev_s *dev, bool lock);
static uint32_t ssp_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency);
static void     ssp_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     ssp_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t ssp_send(struct spi_dev_s *dev, uint32_t wd);
static void     ssp_sndblock(struct spi_dev_s *dev,
                             const void *buffer, size_t nwords);
static void     ssp_recvblock(struct spi_dev_s *dev, void *buffer,
                              size_t nwords);

/* Initialization */

#ifdef CONFIG_LPC17_40_SSP0
static inline struct lpc17_40_sspdev_s *lpc17_40_ssp0initialize(void);
#endif
#ifdef CONFIG_LPC17_40_SSP1
static inline struct lpc17_40_sspdev_s *lpc17_40_ssp1initialize(void);
#endif
#ifdef CONFIG_LPC17_40_SSP2
static inline struct lpc17_40_sspdev_s *lpc17_40_ssp2initialize(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SSP0
static const struct spi_ops_s g_spi0ops =
{
  .lock              = ssp_lock,
  .select            = lpc17_40_ssp0select,   /* Provided externally */
  .setfrequency      = ssp_setfrequency,
  .setmode           = ssp_setmode,
  .setbits           = ssp_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                  /* Not supported */
#endif
  .status            = lpc17_40_ssp0status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc17_40_ssp0cmddata,  /* Provided externally */
#endif
  .send              = ssp_send,
  .sndblock          = ssp_sndblock,
  .recvblock         = ssp_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc17_40_ssp0register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct lpc17_40_sspdev_s g_ssp0dev =
{
  .spidev            =
    {
      &g_spi0ops
    },
  .sspbase           = LPC17_40_SSP0_BASE,
#ifdef CONFIG_LPC17_40_SSP_INTERRUPTS
  .sspirq            = LPC17_40_IRQ_SSP0,
#endif
  .lock              = NXMUTEX_INITIALIZER,
};
#endif /* CONFIG_LPC17_40_SSP0 */

#ifdef CONFIG_LPC17_40_SSP1
static const struct spi_ops_s g_spi1ops =
{
  .lock              = ssp_lock,
  .select            = lpc17_40_ssp1select,   /* Provided externally */
  .setfrequency      = ssp_setfrequency,
  .setmode           = ssp_setmode,
  .setbits           = ssp_setbits,
  .status            = lpc17_40_ssp1status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc17_40_ssp1cmddata,  /* Provided externally */
#endif
  .send              = ssp_send,
  .sndblock          = ssp_sndblock,
  .recvblock         = ssp_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc17_40_ssp1register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct lpc17_40_sspdev_s g_ssp1dev =
{
  .spidev            =
    {
      &g_spi1ops
    },
  .sspbase           = LPC17_40_SSP1_BASE,
#ifdef CONFIG_LPC17_40_SSP_INTERRUPTS
  .sspirq            = LPC17_40_IRQ_SSP1,
#endif
  .lock              = NXMUTEX_INITIALIZER,
};
#endif /* CONFIG_LPC17_40_SSP1 */

#ifdef CONFIG_LPC17_40_SSP2
static const struct spi_ops_s g_spi2ops =
{
  .lock              = ssp_lock,
  .select            = lpc17_40_ssp2select,   /* Provided externally */
  .setfrequency      = ssp_setfrequency,
  .setmode           = ssp_setmode,
  .setbits           = ssp_setbits,
  .status            = lpc17_40_ssp2status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc17_40_ssp2cmddata,  /* Provided externally */
#endif
  .send              = ssp_send,
  .sndblock          = ssp_sndblock,
  .recvblock         = ssp_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc17_40_ssp2register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct lpc17_40_sspdev_s g_ssp2dev =
{
  .spidev            =
    {
      &g_spi2ops
    },
  .sspbase           = LPC17_40_SSP2_BASE,
#ifdef CONFIG_LPC17_40_SSP_INTERRUPTS
  .sspirq            = LPC17_40_IRQ_SSP2,
#endif
  .lock              = NXMUTEX_INITIALIZER,
};
#endif /* CONFIG_LPC17_40_SSP2 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssp_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t ssp_getreg(struct lpc17_40_sspdev_s *priv,
                                  uint8_t offset)
{
  return getreg32(priv->sspbase + (uint32_t)offset);
}

/****************************************************************************
 * Name: ssp_putreg
 *
 * Description:
 *   Write a 32-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ssp_putreg(struct lpc17_40_sspdev_s *priv,
                              uint8_t offset, uint32_t value)
{
  putreg32(value, priv->sspbase + (uint32_t)offset);
}

/****************************************************************************
 * Name: ssp_lock
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

static int ssp_lock(struct spi_dev_s *dev, bool lock)
{
  struct lpc17_40_sspdev_s *priv = (struct lpc17_40_sspdev_s *)dev;
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
 * Name: ssp_setfrequency
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

static uint32_t ssp_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  struct lpc17_40_sspdev_s *priv = (struct lpc17_40_sspdev_s *)dev;
  uint32_t cpsdvsr;
  uint32_t scr;
  uint32_t regval;
  uint32_t actual;

  DEBUGASSERT(priv && frequency <= SSP_CLOCK / 2);

  /* Check if the requested frequency is the same as the frequency
   * selection.
   */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* The SSP bit frequency is given by:
   *
   *   frequency = SSP_CLOCK / (CPSDVSR * (SCR+1)).
   *
   * Let's try for a solution with the smallest value of SCR.  NOTES:
   * (1) In the calculations below, the value of the variable 'scr' is
   * (SCR+1) in the above equation. (2) On slower LPC17xx/LPC40xx parts, SCR
   * will probably always be zero.
   */

  for (scr = 1; scr <= 256; scr++)
    {
      /* CPSDVSR = SSP_CLOCK / (SCR + 1) / frequency */

      cpsdvsr = SSP_CLOCK / (scr * frequency);

      /* Break out on the first solution we find with the smallest value
       * of SCR and with CPSDVSR within the maximum range or 254.
       */

      if (cpsdvsr < 255)
        {
          break;
        }
    }

  DEBUGASSERT(scr <= 256 && cpsdvsr <= 255);

  /* "In master mode, CPSDVSRmin = 2 or larger (even numbers only)" */

  if (cpsdvsr < 2)
    {
      /* Clip to the minimum value. */

      cpsdvsr = 2;
    }
  else if (cpsdvsr > 254)
    {
      /* This should never happen */

      cpsdvsr = 254;
    }

  /* Force even */

  cpsdvsr = (cpsdvsr + 1) & ~1;

  /* Save the new CPSDVSR and SCR values */

  ssp_putreg(priv, LPC17_40_SSP_CPSR_OFFSET, cpsdvsr);

  regval  = ssp_getreg(priv, LPC17_40_SSP_CR0_OFFSET);
  regval &= ~SSP_CR0_SCR_MASK;
  regval |= ((scr - 1) << SSP_CR0_SCR_SHIFT);
  ssp_putreg(priv, LPC17_40_SSP_CR0_OFFSET, regval);

  /* Calculate the new actual */

  actual = SSP_CLOCK / (cpsdvsr * scr);

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("Frequency %" PRId32 "->%" PRId32 "\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: ssp_setmode
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

static void ssp_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct lpc17_40_sspdev_s *priv = (struct lpc17_40_sspdev_s *)dev;
  uint32_t regval;

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CR0 appropriately */

      regval = ssp_getreg(priv, LPC17_40_SSP_CR0_OFFSET);
      regval &= ~(SSP_CR0_CPOL | SSP_CR0_CPHA);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= SSP_CR0_CPHA;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= SSP_CR0_CPOL;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= (SSP_CR0_CPOL | SSP_CR0_CPHA);
          break;

        default:
          spierr("ERROR: Bad mode: %d\n", mode);
          DEBUGASSERT(FALSE);
          return;
        }

      ssp_putreg(priv, LPC17_40_SSP_CR0_OFFSET, regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: ssp_setbits
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

static void ssp_setbits(struct spi_dev_s *dev, int nbits)
{
  struct lpc17_40_sspdev_s *priv = (struct lpc17_40_sspdev_s *)dev;
  uint32_t regval;

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 3 && nbits < 17);

  if (nbits != priv->nbits)
    {
      /* Yes... Set CR1 appropriately */

      regval = ssp_getreg(priv, LPC17_40_SSP_CR0_OFFSET);
      regval &= ~SSP_CR0_DSS_MASK;
      regval |= ((nbits - 1) << SSP_CR0_DSS_SHIFT);
      ssp_putreg(priv, LPC17_40_SSP_CR0_OFFSET, regval);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: ssp_send
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

static uint32_t ssp_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct lpc17_40_sspdev_s *priv = (struct lpc17_40_sspdev_s *)dev;
  register uint32_t regval;

  /* Wait while the TX FIFO is full */

  while (!(ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET) & SSP_SR_TNF));

  /* Write the byte to the TX FIFO */

  ssp_putreg(priv, LPC17_40_SSP_DR_OFFSET, wd);

  /* Wait for the RX FIFO not empty */

  while (!(ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET) & SSP_SR_RNE));

  /* Get the value from the RX FIFO and return it */

  regval = ssp_getreg(priv, LPC17_40_SSP_DR_OFFSET);
  spiinfo("%04" PRId32 "->%04" PRId32 "\n", wd, regval);
  return regval;
}

/****************************************************************************
 * Name: ssp_sndblock
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

static void ssp_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords)
{
  struct lpc17_40_sspdev_s *priv = (struct lpc17_40_sspdev_s *)dev;
  union
  {
    const uint8_t  *p8;
    const uint16_t *p16;
    const void     *pv;
  } u;

  uint32_t data;
  uint32_t sr;

  /* Loop while there are bytes remaining to be sent */

  spiinfo("nwords: %d\n", nwords);
  u.pv = buffer;
  while (nwords > 0)
    {
      /* While the TX FIFO is not full and there are bytes left to send */

      while ((ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET) & SSP_SR_TNF) &&
             nwords)
        {
          /* Fetch the data to send */

          if (priv->nbits > 8)
            {
              data = (uint32_t)*u.p16++;
            }
          else
            {
              data = (uint32_t)*u.p8++;
            }

          /* Send the data */

          ssp_putreg(priv, LPC17_40_SSP_DR_OFFSET, data);
          nwords--;
        }
    }

  /* Then discard all card responses until the RX & TX FIFOs are emptied. */

  spiinfo("discarding\n");
  do
    {
      /* Is there anything in the RX fifo? */

      sr = ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET);
      if ((sr & SSP_SR_RNE) != 0)
        {
          /* Yes.. Read and discard */

          ssp_getreg(priv, LPC17_40_SSP_DR_OFFSET);
        }

      /* There is a race condition where TFE may go true just before
       * RNE goes true and this loop terminates prematurely.  The nasty
       * little delay in the following solves that (it could probably be
       * tuned to improve performance).
       */

      else if ((sr & SSP_SR_TFE) != 0)
        {
          up_udelay(100);
          sr = ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET);
        }
    }
  while ((sr & SSP_SR_RNE) != 0 || (sr & SSP_SR_TFE) == 0);
}

/****************************************************************************
 * Name: ssp_recvblock
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
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssp_recvblock(struct spi_dev_s *dev, void *buffer,
                          size_t nwords)
{
  struct lpc17_40_sspdev_s *priv = (struct lpc17_40_sspdev_s *)dev;
  union
  {
    uint8_t  *p8;
    uint16_t *p16;
    void     *pv;
  } u;

  uint32_t data;
  uint32_t rxpending = 0;

  /* While there is remaining to be sent (and no synchronization error
   * has occurred)
   */

  spiinfo("nwords: %zd\n", nwords);
  u.pv = buffer;
  while (nwords || rxpending)
    {
      /* Fill the transmit FIFO with 0xffff...
       * Write 0xff to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      spiinfo("TX: rxpending: %" PRId32 " nwords: %zd\n", rxpending, nwords);
      while ((ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET) & SSP_SR_TNF) &&
             (rxpending < LPC17_40_SSP_FIFOSZ) && nwords)
        {
          ssp_putreg(priv, LPC17_40_SSP_DR_OFFSET, 0xffff);
          nwords--;
          rxpending++;
        }

      /* Now, read the RX data from the RX FIFO while the RX FIFO is
       * not empty.
       */

      spiinfo("RX: rxpending: %" PRId32 "\n", rxpending);
      while (ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET) & SSP_SR_RNE)
        {
          data = (uint8_t)ssp_getreg(priv, LPC17_40_SSP_DR_OFFSET);
          if (priv->nbits > 8)
            {
              *u.p16++ = (uint16_t)data;
            }
          else
            {
              *u.p8++  = (uint8_t)data;
            }

          rxpending--;
        }
    }
}

/****************************************************************************
 * Name: lpc17_40_ssp0initialize
 *
 * Description:
 *   Initialize the SSP0
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SSP0
static inline struct lpc17_40_sspdev_s *lpc17_40_ssp0initialize(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Configure multiplexed pins as connected on the board.  Chip select
   * pins must be configured by board-specific logic.  All SSP0 pins and
   * one SSP1 pin (SCK) have multiple, alternative pin selection.
   * Definitions in the board.h file must be provided to resolve the
   * board-specific pin configuration like:
   *
   * #define GPIO_SSP0_SCK GPIO_SSP0_SCK_1
   */

  flags = enter_critical_section();
  lpc17_40_configgpio(GPIO_SSP0_SCK);
  lpc17_40_configgpio(GPIO_SSP0_MISO);
  lpc17_40_configgpio(GPIO_SSP0_MOSI);

  /* Configure clocking */

#ifdef LPC176x
  regval  = getreg32(LPC17_40_SYSCON_PCLKSEL1);
  regval &= ~SYSCON_PCLKSEL1_SSP0_MASK;
  regval |= (SSP_PCLKSET_DIV << SYSCON_PCLKSEL1_SSP0_SHIFT);
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL1);
#endif

  /* Enable peripheral clocking to SSP0 */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCSSP0;
  putreg32(regval, LPC17_40_SYSCON_PCONP);
  leave_critical_section(flags);

  return &g_ssp0dev;
}
#endif

/****************************************************************************
 * Name: lpc17_40_ssp1initialize
 *
 * Description:
 *   Initialize the SSP1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SSP1
static inline struct lpc17_40_sspdev_s *lpc17_40_ssp1initialize(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Configure multiplexed pins as connected on the board.  Chip select
   * pins must be configured by board-specific logic.  All SSP0 pins and
   * one SSP1 pin (SCK) have multiple, alternative pin selection.
   * Definitions in the board.h file must be provided to resolve the
   * board-specific pin configuration like:
   *
   * #define GPIO_SSP0_SCK GPIO_SSP0_SCK_1
   */

  flags = enter_critical_section();
  lpc17_40_configgpio(GPIO_SSP1_SCK);
  lpc17_40_configgpio(GPIO_SSP1_MISO);
  lpc17_40_configgpio(GPIO_SSP1_MOSI);

  /* Configure clocking */

#ifdef LPC176x
  regval  = getreg32(LPC17_40_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_SSP1_MASK;
  regval |= (SSP_PCLKSET_DIV << SYSCON_PCLKSEL0_SSP1_SHIFT);
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);
#endif

  /* Enable peripheral clocking to SSP0 and SSP1 */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCSSP1;
  putreg32(regval, LPC17_40_SYSCON_PCONP);
  leave_critical_section(flags);

  return &g_ssp1dev;
}
#endif

/****************************************************************************
 * Name: lpc17_40_ssp2initialize
 *
 * Description:
 *   Initialize the SSP2
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SSP2
static inline struct lpc17_40_sspdev_s *lpc17_40_ssp2initialize(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Configure multiplexed pins as connected on the board.  Chip select
   * pins must be configured by board-specific logic.  All SSP2 pins have
   * multiple, alternative pin selection. Definitions in the board.h file
   * must be provided to resolve the board-specific pin configuration like:
   *
   * #define GPIO_SSP2_SCK GPIO_SSP2_SCK_1
   */

  flags = enter_critical_section();
  lpc17_40_configgpio(GPIO_SSP2_SCK);
  lpc17_40_configgpio(GPIO_SSP2_MISO);
  lpc17_40_configgpio(GPIO_SSP2_MOSI);

  /* Configure clocking */

#ifdef LPC176x
  regval  = getreg32(LPC17_40_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_SSP2_MASK;
  regval |= (SSP_PCLKSET_DIV << SYSCON_PCLKSEL0_SSP2_SHIFT);
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);
#endif

  /* Enable peripheral clocking to SSP0 and SSP1 */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCSSP2;
  putreg32(regval, LPC17_40_SYSCON_PCONP);
  leave_critical_section(flags);

  return &g_ssp2dev;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_sspbus_initialize
 *
 * Description:
 *   Initialize the selected SSP port.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *lpc17_40_sspbus_initialize(int port)
{
  struct lpc17_40_sspdev_s *priv;
  uint32_t regval;
  int i;

  /* Only the SSP0 and SSP1 interfaces are supported */

  switch (port)
    {
#ifdef CONFIG_LPC17_40_SSP0
    case 0:
      priv = lpc17_40_ssp0initialize();
      break;
#endif
#ifdef CONFIG_LPC17_40_SSP1
    case 1:
      priv = lpc17_40_ssp1initialize();
      break;
#endif
#ifdef CONFIG_LPC17_40_SSP2
    case 2:
      priv = lpc17_40_ssp2initialize();
      break;
#endif
    default:
      return NULL;
    }

  /* Configure 8-bit SPI mode */

  ssp_putreg(priv, LPC17_40_SSP_CR0_OFFSET,
             SSP_CR0_DSS_8BIT | SSP_CR0_FRF_SPI);

  /* Disable the SSP and all interrupts (we'll poll for all data) */

  ssp_putreg(priv, LPC17_40_SSP_CR1_OFFSET, 0);
  ssp_putreg(priv, LPC17_40_SSP_IMSC_OFFSET, 0);

  /* Set the initial SSP configuration */

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  ssp_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Enable the SPI */

  regval = ssp_getreg(priv, LPC17_40_SSP_CR1_OFFSET);
  ssp_putreg(priv, LPC17_40_SSP_CR1_OFFSET, regval | SSP_CR1_SSE);
  for (i = 0; i < LPC17_40_SSP_FIFOSZ; i++)
    {
      ssp_getreg(priv, LPC17_40_SSP_DR_OFFSET);
    }

  return &priv->spidev;
}

/****************************************************************************
 * Name: ssp_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be done
 *   after a device is deselected if you worry about such things.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ssp_flush(struct spi_dev_s *dev)
{
  struct lpc17_40_sspdev_s *priv = (struct lpc17_40_sspdev_s *)dev;

  /* Wait for the TX FIFO not full indication */

  while (!(ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET) & SSP_SR_TNF));
  ssp_putreg(priv, LPC17_40_SSP_DR_OFFSET, 0xff);

  /* Wait until TX FIFO and TX shift buffer are empty */

  while (ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET) & SSP_SR_BSY);

  /* Wait until RX FIFO is not empty */

  while (!(ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET) & SSP_SR_RNE));

  /* Then read and discard bytes until the RX FIFO is empty */

  do
    {
      ssp_getreg(priv, LPC17_40_SSP_DR_OFFSET);
    }
  while (ssp_getreg(priv, LPC17_40_SSP_SR_OFFSET) & SSP_SR_RNE);
}

#endif /* CONFIG_LPC17_40_SSP0/1 */
