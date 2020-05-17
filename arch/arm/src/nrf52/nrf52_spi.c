/****************************************************************************
 * arch/arm/src/nrf52/nrf52_spi.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <arch/board/board.h>

#include "arm_arch.h"

#include "nrf52_gpio.h"
#include "nrf52_spi.h"

#include "hardware/nrf52_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C0/SPI0 and I2C1/SPI1 share the same peripherals */

#if defined(CONFIG_NRF52_I2C0_MASTER) && defined(CONFIG_NRF52_SPI0_MASTER)
#  error Unsupported configuration I2C0 + SPI0
#endif
#if defined(CONFIG_NRF52_I2C1_MASTER) && defined(CONFIG_NRF52_SPI1_MASTER)
#  error Unsupported configuration I2C1 + SPI1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         base;       /* Base address of SPI register */
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  uint32_t         irq;        /* SPI IRQ number */
#endif
  uint32_t         sck_pin;    /* SCK pin configuration */
  uint32_t         mosi_pin;   /* MOSI pin configuration */
  uint32_t         miso_pin;   /* MISO pin configuration */
  uint32_t         frequency;  /* Requested clock frequency */
  uint8_t          mode;       /* Mode 0,1,2,3 */

  sem_t            exclsem;    /* Held while chip is selected for mutual
                                * exclusion
                                */
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  sem_t            sem_isr;    /* Interrupt wait semaphore */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void nrf52_spi_putreg(FAR struct nrf52_spidev_s *priv,
                                    uint32_t offset,
                                    uint32_t value);
static inline uint32_t nrf52_spi_getreg(FAR struct nrf52_spidev_s *priv,
                                        uint32_t offset);

/* SPI methods */

static int nrf52_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t nrf52_spi_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency);
static void nrf52_spi_setmode(FAR struct spi_dev_s *priv,
                              enum spi_mode_e mode);
static void nrf52_spi_setbits(FAR struct spi_dev_s *priv, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int nrf52_spi_hwfeatures(FAR struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint32_t nrf52_spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void nrf52_spi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void nrf52_spi_sndblock(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               size_t nwords);
static void nrf52_spi_recvblock(FAR struct spi_dev_s *dev,
                                FAR void *rxbuffer,
                                size_t nwords);
#endif

#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
static int nrf52_spi_isr(int irq, FAR void *context, FAR void *arg);
#endif

/* Initialization */

static int nrf52_spi_init(FAR struct nrf52_spidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI0 */

#ifdef CONFIG_NRF52_SPI0_MASTER
static const struct spi_ops_s g_spi0ops =
{
  .lock              = nrf52_spi_lock,
  .select            = nrf52_spi0select,
  .setfrequency      = nrf52_spi_setfrequency,
  .setmode           = nrf52_spi_setmode,
  .setbits           = nrf52_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf52_spi_hwfeatures,
#  endif
  .status            = nrf52_spi0status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf52_spi1cmddata,
#  endif
  .send              = nrf52_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf52_spi_exchange,
#  else
  .sendlock          = nrf52_spi_sendblock,
  .recvblock         = nrf52_spi_recvblock
#  endif
};

static struct nrf52_spidev_s g_spi0dev =
{
  .spidev    =
  {
    &g_spi0ops
  },

  .base      = NRF52_SPIM0_BASE,
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  .irq       = NRF52_IRQ_SPI_TWI_0,
#endif
  .sck_pin   = BOARD_SPI0_SCK_PIN,
  .mosi_pin  = BOARD_SPI0_MOSI_PIN,
  .miso_pin  = BOARD_SPI0_MISO_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI1 */

#ifdef CONFIG_NRF52_SPI1_MASTER
static const struct spi_ops_s g_spi1ops =
{
  .lock              = nrf52_spi_lock,
  .select            = nrf52_spi1select,
  .setfrequency      = nrf52_spi_setfrequency,
  .setmode           = nrf52_spi_setmode,
  .setbits           = nrf52_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf52_spi_hwfeatures,
#  endif
  .status            = nrf52_spi1status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf52_spi1cmddata,
#  endif
  .send              = nrf52_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf52_spi_exchange,
#  else
  .sendlock          = nrf52_spi_sendblock,
  .recvblock         = nrf52_spi_recvblock
#  endif
};

static struct nrf52_spidev_s g_spi1dev =
{
  .spidev    =
  {
    &g_spi1ops
  },

  .base      = NRF52_SPIM0_BASE,
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  .irq       = NRF52_IRQ_SPI_TWI_1,
#endif
  .sck_pin   = BOARD_SPI1_SCK_PIN,
  .mosi_pin  = BOARD_SPI1_MOSI_PIN,
  .miso_pin  = BOARD_SPI1_MISO_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI2 */

#ifdef CONFIG_NRF52_SPI2_MASTER
static const struct spi_ops_s g_spi2ops =
{
  .lock              = nrf52_spi_lock,
  .select            = nrf52_spi2select,
  .setfrequency      = nrf52_spi_setfrequency,
  .setmode           = nrf52_spi_setmode,
  .setbits           = nrf52_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf52_spi_hwfeatures,
#  endif
  .status            = nrf52_spi2status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf52_spi1cmddata,
#  endif
  .send              = nrf52_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf52_spi_exchange,
#  else
  .sendlock          = nrf52_spi_sendblock,
  .recvblock         = nrf52_spi_recvblock
#  endif
};

static struct nrf52_spidev_s g_spi2dev =
{
  .spidev    =
  {
    &g_spi2ops
  },

  .base      = NRF52_SPIM0_BASE,
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  .irq       = NRF52_IRQ_SPI2,
#endif
  .sck_pin   = BOARD_SPI2_SCK_PIN,
  .mosi_pin  = BOARD_SPI2_MOSI_PIN,
  .miso_pin  = BOARD_SPI2_MISO_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI3 */

#ifdef CONFIG_NRF52_SPI3_MASTER
static const struct spi_ops_s g_spi3ops =
{
  .lock              = nrf52_spi_lock,
  .select            = nrf52_spi3select,
  .setfrequency      = nrf52_spi_setfrequency,
  .setmode           = nrf52_spi_setmode,
  .setbits           = nrf52_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf52_spi_hwfeatures,
#  endif
  .status            = nrf52_spi3status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf52_spi1cmddata,
#  endif
  .send              = nrf52_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf52_spi_exchange,
#  else
  .sendlock          = nrf52_spi_sendblock,
  .recvblock         = nrf52_spi_recvblock
#  endif
};

static struct nrf52_spidev_s g_spi3dev =
{
  .spidev    =
  {
    &g_spi3ops
  },

  .base      = NRF52_SPIM0_BASE,
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  .irq       = NRF52_IRQ_SPI3,
#endif
  .sck_pin   = BOARD_SPI3_SCK_PIN,
  .mosi_pin  = BOARD_SPI3_MOSI_PIN,
  .miso_pin  = BOARD_SPI3_MISO_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_spi_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void nrf52_spi_putreg(FAR struct nrf52_spidev_s *priv,
                                    uint32_t offset,
                                    uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_spi_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t nrf52_spi_getreg(FAR struct nrf52_spidev_s *priv,
                                        uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_spi_isr
 *
 * Description:
 *   Common SPI interrupt service routine
 *
 ****************************************************************************/

#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
static int nrf52_spi_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct nrf52_spidev_s *priv = (FAR struct nrf52_spidev_s *)arg;
  uint32_t regval = 0;

  /* Get interrupt event */

  if (nrf52_spi_getreg(priv, NRF52_SPIM_EVENTS_END_OFFSET) == 1)
    {
      /* Transfer is complete */

      nxsem_post(&priv->sem_isr);

      /* Clear event */

      nrf52_spi_putreg(priv, NRF52_SPIM_EVENTS_END_OFFSET, 0);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: nrf52_spi_init
 *
 * Description:
 *   Configure SPI
 *
 ****************************************************************************/

static int nrf52_spi_init(FAR struct nrf52_spidev_s *priv)
{
  uint32_t regval = 0;
  int      pin    = 0;
  int      port   = 0;

  /* Disable SPI */

  nrf52_spi_putreg(priv, NRF52_SPIM_ENABLE_OFFSET, SPIM_ENABLE_DIS);

  /* Configure SPI pins */

  nrf52_gpio_config(priv->sck_pin);
  nrf52_gpio_config(priv->mosi_pin);
  nrf52_gpio_config(priv->miso_pin);

  /* Select SCK pins */

  pin  = GPIO_PIN_DECODE(priv->sck_pin);
  port = GPIO_PORT_DECODE(priv->sck_pin);

  regval = (pin << SPIM_PSELSCK_PIN_SHIFT);
  regval |= (port << SPIM_PSELSCK_PORT_SHIFT);
  nrf52_spi_putreg(priv, NRF52_SPIM_PSELSCK_OFFSET, regval);

  /* Select MOSI pins */

  pin  = GPIO_PIN_DECODE(priv->mosi_pin);
  port = GPIO_PORT_DECODE(priv->mosi_pin);

  regval = (pin << SPIM_PSELMOSI_PIN_SHIFT);
  regval |= (port << SPIM_PSELMOSI_PORT_SHIFT);
  nrf52_spi_putreg(priv, NRF52_SPIM_PSELMOSI_OFFSET, regval);

  /* According to manual we have to write 0 to MOSI pin */

  nrf52_gpio_write(priv->mosi_pin, false);

  /* Select MISO pins */

  pin   = GPIO_PIN_DECODE(priv->miso_pin);
  port  = GPIO_PORT_DECODE(priv->miso_pin);

  regval = (pin << SPIM_PSELMISO_PIN_SHIFT);
  regval |= (port << SPIM_PSELMISO_PORT_SHIFT);
  nrf52_spi_putreg(priv, NRF52_SPIM_PSELMISO_OFFSET, regval);

  /* NOTE: Chip select pin must be configured by board-specific logic */

#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  /* Enable interrupts for RX and TX done */

  regval = SPIM_INT_END;
  nrf52_spi_putreg(priv, NRF52_SPIM_INTENSET_OFFSET, regval);
#endif

  /* Enable SPI */

  nrf52_spi_putreg(priv, NRF52_SPIM_ENABLE_OFFSET, SPIM_ENABLE_EN);

  return OK;
}

/****************************************************************************
 * Name: nrf52_spi_lock
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

static int nrf52_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct nrf52_spidev_s *priv = (FAR struct nrf52_spidev_s *)dev;
  int ret = OK;

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
 * Name: nrf52_spi_setfrequency
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

static uint32_t nrf52_spi_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  FAR struct nrf52_spidev_s *priv = (FAR struct nrf52_spidev_s *)dev;
  uint32_t regval = 0;

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency */

      return priv->frequency;
    }

  /* Frequencies are hardcoded */

  switch (frequency)
    {
      case 125000:
      {
        regval = SPIM_FREQUENCY_125KBPS;
        break;
      }

      case 250000:
        {
          regval = SPIM_FREQUENCY_250KBPS;
          break;
        }

      case 500000:
        {
          regval = SPIM_FREQUENCY_500KBPS;
          break;
        }

      case 1000000:
        {
          regval = SPIM_FREQUENCY_1MBPS;
          break;
        }

      case 2000000:
      {
        regval = SPIM_FREQUENCY_2MBPS;
        break;
      }

      case 4000000:
        {
          regval = SPIM_FREQUENCY_4MBPS;
          break;
        }

      case 8000000:
        {
          regval = SPIM_FREQUENCY_8MBPS;
          break;
        }

      default:
        {
          spierr("Frequency unsupported %d\n", frequency);
          goto errout;
        }
    }

  /* Write register */

  nrf52_spi_putreg(priv, NRF52_SPIM_FREQUENCY_OFFSET, regval);

  /* Save the frequency setting */

  priv->frequency = frequency;

  spiinfo("Frequency %d\n", frequency);

errout:
  return priv->frequency;
}

/****************************************************************************
 * Name: nrf52_spi_setmode
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
 ****************************************************************************/

static void nrf52_spi_setmode(FAR struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  FAR struct nrf52_spidev_s *priv = (FAR struct nrf52_spidev_s *)dev;
  uint32_t regval = 0;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      regval = nrf52_spi_getreg(priv, NRF52_SPIM_CONFIG_OFFSET);
      regval &= ~(SPIM_CONFIG_CPHA | SPIM_CONFIG_CPOL);

      switch (mode)
        {
          case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
            {
              break;
            }

          case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
            {
              regval |= SPIM_CONFIG_CPHA;
              break;
            }

          case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
            {
              regval |= SPIM_CONFIG_CPOL;
              break;
            }

          case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
            {
              regval |= SPIM_CONFIG_CPHA;
              regval |= SPIM_CONFIG_CPOL;
              break;
            }

          default:
            {
              DEBUGASSERT(0);
              return;
            }
        }

      /* According to manual we have to set SCK pin output
       * value the same as CPOL value
       */

      if (mode == SPIDEV_MODE2 || mode == SPIDEV_MODE3)
        {
          nrf52_gpio_write(priv->sck_pin, true);
        }
      else
        {
          nrf52_gpio_write(priv->sck_pin, false);
        }

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: nrf52_spi_setbits
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

static void nrf52_spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  if (nbits != 8)
    {
      spierr("ERROR: nbits not supported: %d\n", nbits);
    }

  return;
}

/****************************************************************************
 * Name: nrf52_spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int nrf52_spi_hwfeatures(FAR struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  FAR struct nrf52_spidev_s *priv = (FAR struct nrf52_spidev_s *)dev;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  spiinfo("features=%08x\n", features);

  /* Transfer data LSB first? */

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      setbits = SPIM_CONFIG_ORDER;
      clrbits = 0;
    }
  else
    {
      setbits = 0;
      clrbits = SPIM_CONFIG_ORDER;
    }

  regval = nrf52_spi_getreg(priv, NRF52_SPIM_CONFIG_OFFSET);
  regval &= ~clrbits;
  regval |= setbits;
  nrf52_spi_putreg(priv, NRF52_SPIM_CONFIG_OFFSET, regval);

#endif
  /* Other H/W features are not supported */

  return ((features & ~HWFEAT_LSBFIRST) == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: n4f52_spi_send
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

static uint32_t nrf52_spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  uint32_t ret = 0;

  /* Exchange one word on SPI */

  nrf52_spi_exchange(dev, &wd, &ret, 1);

  return ret;
}

/****************************************************************************
 * Name: nrf52_spi_exchange
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
 *              selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf52_spi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
{
  FAR struct nrf52_spidev_s *priv = (FAR struct nrf52_spidev_s *)dev;
  uint32_t regval = 0;

  if (rxbuffer != NULL)
    {
      /* Write RXD data pointer */

      regval = (uint32_t)rxbuffer;
      nrf52_spi_putreg(priv, NRF52_SPIM_RXDPTR_OFFSET, regval);

      /* Write number of bytes in RXD buffer */

      regval = nwords;
      nrf52_spi_putreg(priv, NRF52_SPIM_RXDMAXCNT_OFFSET, regval);
    }

  if (txbuffer != NULL)
    {
      /* Write TXD data pointer */

      regval = (uint32_t)txbuffer;
      nrf52_spi_putreg(priv, NRF52_SPIM_TXDPTR_OFFSET, regval);

      /* Write number of bytes in TXD buffer */

      regval = nwords;
      nrf52_spi_putreg(priv, NRF52_SPIM_TXDMAXCNT_OFFSET, regval);
    }

  /* SPI start */

  nrf52_spi_putreg(priv, NRF52_SPIM_TASK_START_OFFSET, SPIM_TASKS_START);

#ifndef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  /* Wait for RX done and TX done */

  while (nrf52_spi_getreg(priv, NRF52_SPIM_EVENTS_END_OFFSET) != 1);

  /* Clear event */

  nrf52_spi_putreg(priv, NRF52_SPIM_EVENTS_END_OFFSET, 0);
#else
  /* Wait for transfer complete */

  nxsem_wait(&priv->sem_isr);
#endif

  /* SPI stop */

  nrf52_spi_putreg(priv, NRF52_SPIM_TASK_STOP_OFFSET, SPIM_TASKS_STOP);

  /* Wait for STOP event */

  while (nrf52_spi_getreg(priv, NRF52_SPIM_EVENTS_STOPPED_OFFSET) != 1);

  /* Clear event */

  nrf52_spi_putreg(priv, NRF52_SPIM_EVENTS_STOPPED_OFFSET, 0);

  /* Clear RX/TX DMA after transfer */

  nrf52_spi_putreg(priv, NRF52_SPIM_RXDPTR_OFFSET, 0);
  nrf52_spi_putreg(priv, NRF52_SPIM_RXDMAXCNT_OFFSET, 0);
  nrf52_spi_putreg(priv, NRF52_SPIM_TXDPTR_OFFSET, 0);
  nrf52_spi_putreg(priv, NRF52_SPIM_TXDMAXCNT_OFFSET, 0);
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: nrf52_spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of
 *              words.  The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf52_spi_sndblock(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return nrf52_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: nrf52_spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in
 *              number of words. The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf52_spi_recvblock(FAR struct spi_dev_s *dev,
                                FAR void *rxbuffer,
                                size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return nrf52_spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif /* CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *nrf52_spibus_initialize(int port)
{
  FAR struct nrf52_spidev_s *priv = NULL;

  /* Get SPI driver data */

  switch (port)
    {
#ifdef CONFIG_NRF52_SPI0_MASTER
      case 0:
        {
          priv = &g_spi0dev;
          break;
        }
#endif

#ifdef CONFIG_NRF52_SPI1_MASTER
      case 1:
        {
          priv = &g_spi1dev;
          break;
        }
#endif

#ifdef CONFIG_NRF52_SPI2_MASTER
      case 2:
        {
          priv = &g_spi2dev;
          break;
        }
#endif

#ifdef CONFIG_NRF52_SPI3_MASTER
      case 3:
        {
          priv = &g_spi3dev;
          break;
        }
#endif

      default:
        {
          goto errout;
        }
    }

  /* Initialize the SPI */

  nrf52_spi_init(priv);

  /* Initialize the SPI semaphore */

  nxsem_init(&priv->exclsem, 0, 1);

#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->sem_isr, 0, 0);
  nxsem_set_protocol(&priv->sem_isr, SEM_PRIO_NONE);

  /* Attach SPI interrupt */

  irq_attach(priv->irq, nrf52_spi_isr, priv);
  up_enable_irq(priv->irq);
#endif

errout:
  return (FAR struct spi_dev_s *)priv;
}
