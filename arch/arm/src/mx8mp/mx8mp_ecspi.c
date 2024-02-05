/******************************************************************************
 * arch/arm/src/mx8mp/mx8mp_ecspi.c
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
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>

#include "mx8mp_ecspi.h"
#include "mx8mp_ccm.h"
#include "hardware/mx8mp_memorymap.h"
#include "hardware/mx8mp_ecspi.h"

#include <debug.h>
#include "arm_internal.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

struct mx8mp_spi_s;

static void mx8mp_spi_enable(struct mx8mp_spi_s *dev);
static int mx8mp_spi_interrupt(int irq, void *context, void *arg);
static int mx8mp_spi_transfer(struct mx8mp_spi_s *priv,
                              const uint8_t *out, uint8_t *in, int size);

/* SPI methods */

static int spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(struct spi_dev_s *dev, spi_hwfeatures_t features);
#endif
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void spi_exchange(struct spi_dev_s *dev,
                         const void *txbuffer, void *rxbuffer, size_t nwords);
#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(struct spi_dev_s *dev);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock (struct spi_dev_s *dev,
                          const void *txbuffer, size_t nwords);
static void spi_recvblock(struct spi_dev_s *dev,
                          void *rxbuffer, size_t nwords);
#endif

/* Initialization */

static void        spi_bus_initialize(struct mx8mp_spi_s *priv);

/******************************************************************************
 * Private Types
 ******************************************************************************/

struct mx8mp_spi_s
{
  struct spi_dev_s dev;       /* Externally visible part of the SPI interface */
  uint32_t base;              /* SPI base address */
  int      clock;             /* Peripheral clock as described in hardware/mx8mp_ccm.h */
  uint16_t irq;               /* IRQ number */
  uint32_t frequency;         /* Current desired SCLK frequency */
  uint32_t actual;            /* TODO: check use - Current actual SCLK frequency */
  uint8_t  nbits;             /* Width of word in bits (8 to 16) */
  uint8_t  mode;              /* Mode 0,1,2,3 */
  mutex_t  lock;              /* Only one thread can access at a time */
  sem_t    wait;              /* IRQ wait sync */
  uint32_t byte_duration;     /* Byte duration - depend on clock */
};

/* SPI device structures */

#ifdef CONFIG_MX8MP_SPI1
static const struct spi_ops_s g_spi1_ops =
{
  .lock         = spi_lock,
  .select       = mx8mp_spi1_select,        /* Provided externally by board logic */
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = 0,                        /* Not supported */
#endif
  .status       = mx8mp_spi1_status,        /* Provided externally by board logic */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = mx8mp_spi1_cmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
};

static struct mx8mp_spi_s g_spi1_dev =
{
  .dev.ops    = &g_spi1_ops,
  .base       = MX8M_ECSPI1,
  .clock      = ECSPI1_CLK_ROOT,
  .irq        = MX8MP_IRQ_ECSPI1,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_MX8MP_SPI2
static const struct spi_ops_s g_spi2_ops =
{
  .lock         = spi_lock,
  .select       = mx8mp_spi2_select,        /* Provided externally by board logic */
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = 0,                        /* Not supported */
#endif
  .status       = mx8mp_spi2_status,        /* Provided externally by board logic */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = mx8mp_spi2_cmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
};

static struct mx8mp_spi_s g_spi1_dev =
{
  .dev.ops    = &g_spi2_ops,
  .base       = MX8M_ECSPI2,
  .clock      = ECSPI2_CLK_ROOT,
  .irq        = MX8MP_IRQ_ECSPI2,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = SEM_INITIALIZER(0),
};
#endif

#ifdef CONFIG_MX8MP_SPI3
static const struct spi_ops_s g_spi3_ops =
{
  .lock         = spi_lock,
  .select       = mx8mp_spi3_select,        /* Provided externally by board logic */
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = 0,                        /* Not supported */
#endif
  .status       = mx8mp_spi3_status,        /* Provided externally by board logic */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = mx8mp_spi3_cmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
};

static struct mx8mp_spi_s g_spi3_dev =
{
  .dev.ops    = &g_spi3_ops,
  .base       = MX8M_ECSPI3,
  .clock      = ECSPI3_CLK_ROOT,
  .irq        = MX8MP_IRQ_ECSPI3,
  .lock       = NXMUTEX_INITIALIZER,
  .wait       = SEM_INITIALIZER(0),
};
#endif

/******************************************************************************
 * Private Data
 ******************************************************************************/

/******************************************************************************
 * Private Functions
 ******************************************************************************/

static void mx8mp_spi_enable(struct mx8mp_spi_s *priv)
{
  /* enable and set all channels to master mode */

  modreg32(CONREG_EN, CONREG_EN, priv->base + CONREG_OFFSET);
  modreg32(0xf << CONREG_CHANNEL_MODE,
           0xf << CONREG_CHANNEL_MODE, priv->base + CONREG_OFFSET);

  /* select channel 0 (the only one available) */

  modreg32(0, 0x3 << CONREG_CHANNEL_SELECT, priv->base + CONREG_OFFSET);
}

/******************************************************************************
 * Name: mx8mp_spi_wait_irq
 *
 * Description:
 *   Wait for the IRQ to be triggered.
 *
 ******************************************************************************/

static inline int mx8mp_spi_wait_irq(struct mx8mp_spi_s *priv, int nsec)
{
  return nxsem_tickwait(&priv->wait, NSEC2TICK(nsec));
}

/******************************************************************************
 * Name: mx8mp_spi_interrupt
 *
 * Description:
 *   The SPI common interrupt handler
 *
 ******************************************************************************/

static int mx8mp_spi_interrupt(int irq, void *context, void *arg)
{
  struct mx8mp_spi_s *priv = (struct mx8mp_spi_s *)arg;
  putreg32(0, priv->base + INTREG_OFFSET);
  nxsem_post(&priv->wait);

  return 0;
}

/******************************************************************************
 * Name: mx8mp_spi_transfer
 *
 * Description:
 *   Do a transfer on the SPI bus. It may be needed to call it multiple times
 *   since SPI FIFO is limited to 256 bytes (64 x 32bits)
 *
 ******************************************************************************/

static int mx8mp_spi_transfer(struct mx8mp_spi_s *priv,
                              const uint8_t *out, uint8_t *in, int size)
{
  /* max 256 bytes per access (64 x 32 bits) */

  if (size > 256)
    {
      size = 256;
    }

  /* WARNING: SPI FIFO works on 32 bits data only */

  /* 1. clear and enable IRQs */

  putreg32(INTREG_RDREN, priv->base + INTREG_OFFSET);

  modreg32(STATREG_TC | STATREG_RO,
           STATREG_TC | STATREG_RO,
           priv->base + STATREG_OFFSET);

  /* 2. compute words to process */

  uint16_t words = size / 4;
  uint16_t remaining_bytes = size % 4;

  /* 3. Configure burst size (in bits) and RX IT threshold (in words) */

  uint16_t const burst_length = ((words * 4 + remaining_bytes) * 8) & 0xfff;
  modreg32((burst_length - 1) << CONREG_BURST_LENGTH,
            0xfff << CONREG_BURST_LENGTH,
            priv->base + CONREG_OFFSET);

  uint16_t threshold = words;
  if (remaining_bytes)
    {
      ++threshold;
    }

  putreg32((threshold - 1) << DMAREG_RX_THRESHOLD, priv->base + DMAREG_OFFSET);

  /* 4. Load TX FIFO - remaining byte on the first word */

  if (remaining_bytes)
    {
      uint32_t data = 0;
      for (int32_t i = 0; i < remaining_bytes; ++i)
        {
          data = (data << 8) | out[i];
        }

      putreg32(data, priv->base + TXDATA_OFFSET);
    }

  for (int32_t i = 0; i < words; ++i)
    {
      uint32_t data = 0;
      memcpy(&data, out + i * 4 + remaining_bytes, 4);
      data = __builtin_bswap32(data);
      putreg32(data, priv->base + TXDATA_OFFSET);
    }

  /* 5. start transfer and wait for it */

  modreg32(CONREG_XCH, CONREG_XCH, priv->base + CONREG_OFFSET);
  int ret = mx8mp_spi_wait_irq(priv, priv->byte_duration * (size + 1)); /* add one byte duration of leeway */
  if (ret < 0)
    {
      return ret;
    }

  /* 6. Unload RX FIFO (even if an error occured to empty the queue) */

  if (remaining_bytes)
    {
      uint32_t data = getreg32(priv->base + RXDATA_OFFSET);
      data = __builtin_bswap32(data);
      data >>= (4 - remaining_bytes) * 8;
      memcpy(in, &data, remaining_bytes);
    }

  for (int32_t i = 0; i < words; ++i)
    {
      uint32_t data = getreg32(priv->base + RXDATA_OFFSET);
      data = __builtin_bswap32(data);
      memcpy(in + i * 4 + remaining_bytes, &data, 4);
    }

  return size;
}

/******************************************************************************
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
 ******************************************************************************/

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct mx8mp_spi_s *priv = (struct mx8mp_spi_s *)dev;
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

/******************************************************************************
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
 ******************************************************************************/

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct mx8mp_spi_s *priv = (struct mx8mp_spi_s *)dev;

  if (frequency == priv->frequency)
    {
      return priv->actual;
    }

  uint32_t clock = mx8mp_ccm_get_clock(priv->clock);
  DEBUGASSERT(clock > frequency);

  uint8_t pre_div  = 0;
  uint8_t post_div = 0;
  uint32_t old_frequency = 0;

  for (uint8_t i = 0; i < 16; ++i)
    {
      for (uint8_t j = 0; j < 16; ++j)
        {
          uint32_t current_frequency = clock / ((1 << i) * (j + 1));
          if ((current_frequency <= frequency) &&
              (current_frequency > old_frequency))
            {
              pre_div  = j;
              post_div = i;
              old_frequency = current_frequency;
            }
        }
    }

  priv->actual = clock / ((1 << post_div) * (pre_div + 1));

  spiinfo("Source clock:   %lu", clock);
  spiinfo("Requested freq: %lu", frequency);
  spiinfo("pre_divider:    %u",  pre_div);
  spiinfo("post_divider:   %u",  post_div);
  spiinfo("Actual freq:    %lu", priv->actual);

  modreg32(pre_div  << CONREG_PRE_DIVIDER,  0xf << CONREG_PRE_DIVIDER,
           priv->base + CONREG_OFFSET);
  modreg32(post_div << CONREG_POST_DIVIDER, 0xf << CONREG_POST_DIVIDER,
           priv->base + CONREG_OFFSET);

  /* Store byte duration to adapt irq waiting time */

  priv->byte_duration = (NSEC_PER_SEC / priv->actual) * 8;

  return priv->actual;
}

/******************************************************************************
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
 *   Returns the actual frequency selected
 *
 ******************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct mx8mp_spi_s *priv = (struct mx8mp_spi_s *)dev;
  uint8_t CPOL;
  uint8_t CPHA;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode == priv->mode)
    {
      return;
    }

  switch (mode)
    {
    case SPIDEV_MODE0:
      {
        CPOL = 0;
        CPHA = 0;
      }
      break;

    case SPIDEV_MODE1:
      {
        CPOL = 0;
        CPHA = 1;
      }
      break;

    case SPIDEV_MODE2:
      {
        CPOL = 1;
        CPHA = 0;
      }
      break;

    case SPIDEV_MODE3:
      {
        CPOL = 1;
        CPHA = 1;
      }
      break;

    default:
      return;
    }

  uint32_t configreg = 0;
  configreg |= (0xf << CONFIGREG_SLCK_CTL);
  configreg |= (0xf << CONFIGREG_DATA_CTL);
  configreg |= (0x0 << CONFIGREG_SS_POL);
  configreg |= (0x0 << CONFIGREG_SS_CTL);
  configreg |= (CPOL << CONFIGREG_SCLK_POL);
  configreg |= (CPHA << CONFIGREG_SCLK_PHA);
  putreg32(configreg, priv->base + CONFIGREG_OFFSET);

  /* Save the mode so that subsequent re-configurations will be
   * faster
   */

  priv->mode = mode;
}

/******************************************************************************
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
 ******************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct mx8mp_spi_s *priv = (struct mx8mp_spi_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits == priv->nbits)
    {
      return;
    }

  modreg32(nbits << CONREG_BURST_LENGTH,
           0xfff << CONREG_BURST_LENGTH,
           priv->base + CONREG_OFFSET);

  /* Save the selection so that subsequent re-configurations will be faster. */

  priv->nbits = nbits;
}

/******************************************************************************
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
 ******************************************************************************/

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct mx8mp_spi_s *priv = (struct mx8mp_spi_s *)dev;
  uint32_t ret;

  DEBUGASSERT(priv && priv->base);

  uint32_t dummy;
  ret = mx8mp_spi_transfer(priv, (const uint8_t *)&wd,
                                 (uint8_t *)&dummy,
                                 priv->nbits % 8);

  spiinfo("Sent: %04" PRIx32 " Return: %04" PRIx32
          " Status: %02" PRIx32 "\n", wd, ret, dummy);

  return ret;
}

/******************************************************************************
 * Name: spi_exchange (no DMA)
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct mx8mp_spi_s *priv = (struct mx8mp_spi_s *)dev;

  int size = nwords;
  if (priv->nbits > 8)
    {
      size = size * 2;
    }

  int transfered = 0;
  while (transfered < size)
    {
      int ret = mx8mp_spi_transfer(priv,
                                  (const uint8_t *)txbuffer + transfered,
                                        (uint8_t *)rxbuffer + transfered,
                                    size - transfered);
      if (ret < 0)
        {
          break;
        }

      transfered += ret;
    }
}

/******************************************************************************
 * Name: spi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

static void spi_bus_initialize(struct mx8mp_spi_s *priv)
{
  priv->frequency = 0;
  priv->nbits     = 0;
  priv->mode      = SPIDEV_MODE1; /* Ensure that mode0 is applied */

  /* Enable SPI */

  mx8mp_spi_enable(priv);

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Apply a default configuration */

  spi_setbits((struct spi_dev_s *)priv, 8);
  spi_setmode((struct spi_dev_s *)priv, SPIDEV_MODE0);

  /* Attach Interrupt Handler */

  irq_attach(priv->irq, mx8mp_spi_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irq);
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: mx8mp_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ******************************************************************************/

struct spi_dev_s *mx8mp_spibus_initialize(int bus)
{
  struct mx8mp_spi_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

  switch (bus)
    {
#ifdef CONFIG_MX8MP_SPI1
    case 1:
      priv = &g_spi1_dev;
      spi_bus_initialize(priv);
      break;
#endif

#ifdef CONFIG_MX8MP_SPI2
    case 2:
      priv = &g_spi2_dev;
      spi_bus_initialize(priv);
      break;
#endif

#ifdef CONFIG_MX8MP_SPI3
    case 3:
      priv = &g_spi3_dev;
      spi_bus_initialize(priv);
      break;
#endif

    default:
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
    }

  leave_critical_section(flags);
  return (struct spi_dev_s *)priv;
}
