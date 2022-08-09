/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_ssp.c
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
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"

#include "lpc43_ssp.h"
#include "lpc43_cgu.h"
#include "lpc43_scu.h"
#include "lpc43_ccu.h"
#include "lpc43_pinconfig.h"

#if defined(CONFIG_LPC43_SSP0) || defined(CONFIG_LPC43_SSP1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the SSP driver */

struct lpc43_sspdev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         sspbase;    /* SPIn base address */
  uint32_t         sspbasefreq;
#ifdef CONFIG_LPC43_SSP_INTERRUPTS
  uint8_t          sspirq;     /* SPI IRQ number */
#endif
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (4 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t ssp_getreg(struct lpc43_sspdev_s *priv,
                                  uint8_t offset);
static inline void ssp_putreg(struct lpc43_sspdev_s *priv,
                              uint8_t offset, uint32_t value);

/* SPI methods */

static int      ssp_lock(struct spi_dev_s *dev, bool lock);
static uint32_t ssp_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency);
static void     ssp_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     ssp_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t ssp_send(struct spi_dev_s *dev, uint32_t wd);
static void     ssp_exchange(struct spi_dev_s *dev,
                             const void *txbuffer, void *rxbuffer,
                             size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void     ssp_sndblock(struct spi_dev_s *dev,
                             const void *buffer, size_t nwords);
static void     ssp_recvblock(struct spi_dev_s *dev, void *buffer,
                              size_t nwords);
#endif

/* Initialization */

#ifdef CONFIG_LPC43_SSP0
static inline struct lpc43_sspdev_s *lpc43_ssp0initialize(void);
#endif
#ifdef CONFIG_LPC43_SSP1
static inline struct lpc43_sspdev_s *lpc43_ssp1initialize(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LPC43_SSP0
static const struct spi_ops_s g_spi0ops =
{
  .lock              = ssp_lock,
  .select            = lpc43_ssp0select,   /* Provided externally */
  .setfrequency      = ssp_setfrequency,
  .setmode           = ssp_setmode,
  .setbits           = ssp_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                  /* Not supported */
#endif
  .status            = lpc43_ssp0status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc43_ssp0cmddata,  /* Provided externally */
#endif
  .send              = ssp_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = ssp_exchange,
#else
  .sndblock          = ssp_sndblock,
  .recvblock         = ssp_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc43_ssp0register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct lpc43_sspdev_s g_ssp0dev =
{
  .spidev            =
    {
      &g_spi0ops
    },
  .sspbase           = LPC43_SSP0_BASE,
  .sspbasefreq       = BOARD_SSP0_BASEFREQ
#ifdef CONFIG_LPC43_SSP_INTERRUPTS
  .sspirq            = LPC43_IRQ_SSP0,
#endif
};
#endif /* CONFIG_LPC43_SSP0 */

#ifdef CONFIG_LPC43_SSP1
static const struct spi_ops_s g_spi1ops =
{
  .lock              = ssp_lock,
  .select            = lpc43_ssp1select,   /* Provided externally */
  .setfrequency      = ssp_setfrequency,
  .setmode           = ssp_setmode,
  .setbits           = ssp_setbits,
  .status            = lpc43_ssp1status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc43_ssp1cmddata,  /* Provided externally */
#endif
  .send              = ssp_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = ssp_exchange,
#else
  .sndblock          = ssp_sndblock,
  .recvblock         = ssp_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc43_ssp1register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct lpc43_sspdev_s g_ssp1dev =
{
  .spidev            =
    {
      &g_spi1ops
    },
  .sspbase           = LPC43_SSP1_BASE,
  .sspbasefreq       = BOARD_SSP1_BASEFREQ
#ifdef CONFIG_LPC43_SSP_INTERRUPTS
  .sspirq            = LPC43_IRQ_SSP1,
#endif
};
#endif /* CONFIG_LPC43_SSP1 */

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

static inline uint32_t ssp_getreg(struct lpc43_sspdev_s *priv,
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

static inline void ssp_putreg(struct lpc43_sspdev_s *priv,
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
  struct lpc43_sspdev_s *priv = (struct lpc43_sspdev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
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
  struct lpc43_sspdev_s *priv = (struct lpc43_sspdev_s *)dev;
  uint32_t divisor;
  uint32_t actual;

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* frequency = SSP_CLOCK / divisor, or divisor = SSP_CLOCK / frequency */

  divisor = priv->sspbasefreq / frequency;

  /* "In master mode, CPSDVSRmin = 2 or larger (even numbers only)" */

  if (divisor < 2)
    {
      divisor = 2;
    }
  else if (divisor > 254)
    {
      divisor = 254;
    }

  divisor = (divisor + 1) & ~1;

  /* Save the new divisor value */

  ssp_putreg(priv, LPC43_SSP_CPSR_OFFSET, divisor);

  /* Calculate the new actual */

  actual = priv->sspbasefreq / divisor;

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
  struct lpc43_sspdev_s *priv = (struct lpc43_sspdev_s *)dev;
  uint32_t regval;

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      spiinfo("Setting mode to %d.\n", mode);

      /* Yes... Set CR0 appropriately */

      regval = ssp_getreg(priv, LPC43_SSP_CR0_OFFSET);
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

      ssp_putreg(priv, LPC43_SSP_CR0_OFFSET, regval);

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
  struct lpc43_sspdev_s *priv = (struct lpc43_sspdev_s *)dev;
  uint32_t regval;

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 3 && nbits < 17);

  if (nbits != priv->nbits)
    {
      spiinfo("Setting bits per word to %d.\n", nbits);

      /* Yes... Set CR1 appropriately */

      regval = ssp_getreg(priv, LPC43_SSP_CR0_OFFSET);
      regval &= ~SSP_CR0_DSS_MASK;
      regval |= ((nbits - 1) << SSP_CR0_DSS_SHIFT);
      ssp_putreg(priv, LPC43_SSP_CR0_OFFSET, regval);
      spiinfo("SSP Control Register 0 (CR0) after setting"
              "DSS: 0x%08" PRIX32 ".\n", regval);

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
  struct lpc43_sspdev_s *priv = (struct lpc43_sspdev_s *)dev;
  register uint32_t regval;

  /* Wait while the TX FIFO is full */

  while (!(ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_TNF));

  /* Write the byte to the TX FIFO */

  ssp_putreg(priv, LPC43_SSP_DR_OFFSET, wd);

  /* Wait for the RX FIFO not empty */

  while (!(ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_RNE));

  /* Get the value from the RX FIFO and return it */

  regval = ssp_getreg(priv, LPC43_SSP_DR_OFFSET);
  spiinfo("%04" PRIx32 "->%04" PRIx32 "\n", wd, regval);
  return regval;
}

/****************************************************************************
 * Name: ssp_exchange
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssp_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct lpc43_sspdev_s *priv = (struct lpc43_sspdev_s *)dev;
  union
  {
    const uint8_t *p8;
    const uint16_t *p16;
    const void *pv;
  } tx;

  union
  {
    uint8_t *p8;
    uint16_t *p16;
    void *pv;
  } rx;

  uint32_t data;
  uint32_t datadummy = (priv->nbits > 8) ? 0xffff : 0xff;
  uint32_t rxpending = 0;

  /* While there is remaining to be sent (and no synchronization error
   * has occurred)
   */

  spiinfo("nwords: %d\n", nwords);

  tx.pv = txbuffer;
  rx.pv = rxbuffer;

  while (nwords || rxpending)
    {
      /* Write data to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      spiinfo("TX: rxpending: %" PRId32 " nwords: %d\n", rxpending, nwords);
      while ((ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_TNF) &&
             (rxpending < LPC43_SSP_FIFOSZ) && nwords)
        {
          if (txbuffer)
            {
              if (priv->nbits > 8)
                {
                  data = (uint32_t)*tx.p16++;
                }
              else
                {
                  data = (uint32_t)*tx.p8++;
                }
            }

          ssp_putreg(priv, LPC43_SSP_DR_OFFSET, txbuffer?data:datadummy);
          nwords--;
          rxpending++;
        }

      /* Now, read the RX data from the RX FIFO while the RX FIFO is not
       * empty.
       */

      spiinfo("RX: rxpending: %" PRId32 "\n", rxpending);
      while (ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_RNE)
        {
          data = ssp_getreg(priv, LPC43_SSP_DR_OFFSET);
          if (rxbuffer)
            {
              if (priv->nbits > 8)
                {
                  *rx.p16++ = (uint16_t)data;
                }
              else
                {
                  *rx.p8++  = (uint8_t)data;
                }
            }

          rxpending--;
        }
    }
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

#ifndef CONFIG_SPI_EXCHANGE
static void ssp_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords)
{
  return ssp_exchange(dev, buffer, NULL, nwords);
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
  return ssp_exchange(dev, NULL, buffer, nwords);
}
#endif /* !CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Name: lpc43_ssp0initialize
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

#ifdef CONFIG_LPC43_SSP0
static inline struct lpc43_sspdev_s *lpc43_ssp0initialize(void)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  /* Configure clocking */

  regval  = getreg32(LPC43_BASE_SSP0_CLK);
  regval &= ~BASE_SSP0_CLK_CLKSEL_MASK;
  regval |= (BOARD_SSP0_CLKSRC | BASE_SSP0_CLK_AUTOBLOCK);
  putreg32(regval, LPC43_BASE_SSP0_CLK);

  /* Clock register */

  regval  = getreg32(LPC43_CCU1_M4_SSP0_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU1_M4_SSP0_CFG);

  /* Clock peripheral */

  regval  = getreg32(LPC43_CCU2_APB0_SSP0_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU2_APB0_SSP0_CFG);

  /* Pin configuration */

  lpc43_pin_config(PINCONF_SSP0_SCK);
  lpc43_pin_config(PINCONF_SSP0_MISO);
  lpc43_pin_config(PINCONF_SSP0_MOSI);

  leave_critical_section(flags);

  return &g_ssp0dev;
}
#endif

/****************************************************************************
 * Name: lpc43_ssp1initialize
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

#ifdef CONFIG_LPC43_SSP1
static inline struct lpc43_sspdev_s *lpc43_ssp1initialize(void)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  /* Configure clocking */

  regval  = getreg32(LPC43_BASE_SSP1_CLK);
  regval &= ~BASE_SSP1_CLK_CLKSEL_MASK;
  regval |= (BOARD_SSP1_CLKSRC | BASE_SSP1_CLK_AUTOBLOCK);
  putreg32(regval, LPC43_BASE_SSP1_CLK);

  /* Clock register */

  regval  = getreg32(LPC43_CCU1_M4_SSP1_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU1_M4_SSP1_CFG);

  /* Clock peripheral */

  regval  = getreg32(LPC43_CCU2_APB2_SSP1_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU2_APB2_SSP1_CFG);

  /* Pins configuration */

#ifdef PINCONF_SSP1_SCK
  /* It is possible this is not configured if CLK0 is being used for
   * clocking SPI.
   */

  lpc43_pin_config(PINCONF_SSP1_SCK);
#endif
  lpc43_pin_config(PINCONF_SSP1_MISO);
  lpc43_pin_config(PINCONF_SSP1_MOSI);

  leave_critical_section(flags);
  return &g_ssp1dev;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_sspbus_initialize
 *
 * Description:
 *   Initialize the selected SSP port (0=SSP0, 1=SSP1)
 *
 * Input Parameters:
 *   port - Port number (0=SSP0, 1=SSP1)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *lpc43_sspbus_initialize(int port)
{
  struct lpc43_sspdev_s *priv;
  uint32_t regval;
  int i;

  /* Only the SSP0 and SSP1 interfaces are supported */

  switch (port)
    {
#ifdef CONFIG_LPC43_SSP0
    case 0:
      priv = lpc43_ssp0initialize();
      break;
#endif

#ifdef CONFIG_LPC43_SSP1
    case 1:
      priv = lpc43_ssp1initialize();
      break;
#endif

    default:
      return NULL;
    }

  /* Configure 8-bit SPI mode */

  ssp_putreg(priv, LPC43_SSP_CR0_OFFSET, SSP_CR0_DSS_8BIT | SSP_CR0_FRF_SPI);

  /* Disable the SSP and all interrupts (we'll poll for all data) */

  ssp_putreg(priv, LPC43_SSP_CR1_OFFSET, 0);
  ssp_putreg(priv, LPC43_SSP_IMSC_OFFSET, 0);

  /* Set the initial SSP configuration */

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  ssp_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);

  /* Enable the SPI */

  regval = ssp_getreg(priv, LPC43_SSP_CR1_OFFSET);
  ssp_putreg(priv, LPC43_SSP_CR1_OFFSET, regval | SSP_CR1_SSE);
  for (i = 0; i < LPC43_SSP_FIFOSZ; i++)
    {
      ssp_getreg(priv, LPC43_SSP_DR_OFFSET);
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
  struct lpc43_sspdev_s *priv = (struct lpc43_sspdev_s *)dev;

  /* Wait for the TX FIFO not full indication */

  while (!(ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_TNF));
  ssp_putreg(priv, LPC43_SSP_DR_OFFSET, 0xff);

  /* Wait until TX FIFO and TX shift buffer are empty */

  while (ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_BSY);

  /* Wait until RX FIFO is not empty */

  while (!(ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_RNE));

  /* Then read and discard bytes until the RX FIFO is empty */

  do
    {
      ssp_getreg(priv, LPC43_SSP_DR_OFFSET);
    }
  while (ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_RNE);
}

#endif /* CONFIG_LPC43_SSP0/1 */
