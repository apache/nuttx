/****************************************************************************
 * arch/arm/src/nrf52/nrf52_spi.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <arch/board/board.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"
#include "barriers.h"

#include "nrf52_gpio.h"
#include "nrf52_spi.h"

#include "hardware/nrf52_spi.h"

#ifdef CONFIG_NRF52_SPI_MASTER_WORKAROUND_1BYTE_TRANSFER
#  include "hardware/nrf52_gpiote.h"
#  include "nrf52_ppi.h"
#endif

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

/* Reserve PPI channel and GPIOTE channel for 1 byte transfer workaround */

#ifdef CONFIG_NRF52_SPI_MASTER_WORKAROUND_1BYTE_TRANSFER
#  define SPI_1B_WORKAROUND_PPI_CHAN    (NRF52_PPI_NUM_CONFIGURABLE_CHANNELS - 1)
#  define SPI_1B_WORKAROUND_GPIOTE_CHAN (7)
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
  nrf52_pinset_t   sck_pin;    /* SCK pin configuration */
  uint32_t         frequency;  /* Requested clock frequency */
  uint8_t          mode;       /* Mode 0,1,2,3 */

  mutex_t          lock;       /* Held while chip is selected for mutual
                                * exclusion
                                */
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  sem_t            sem_isr;    /* Interrupt wait semaphore */
#endif
  bool             initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void nrf52_spi_putreg(struct nrf52_spidev_s *priv,
                                    uint32_t offset,
                                    uint32_t value);
static inline uint32_t nrf52_spi_getreg(struct nrf52_spidev_s *priv,
                                        uint32_t offset);

/* SPI methods */

static int nrf52_spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t nrf52_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
static void nrf52_spi_setmode(struct spi_dev_s *priv,
                              enum spi_mode_e mode);
static void nrf52_spi_setbits(struct spi_dev_s *priv, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int nrf52_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint32_t nrf52_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void nrf52_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void nrf52_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer,
                               size_t nwords);
static void nrf52_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer,
                                size_t nwords);
#endif

#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
static int nrf52_spi_isr(int irq, void *context, void *arg);
#endif

/* Initialization */

static int nrf52_spi_init(struct nrf52_spidev_s *priv);
static void nrf52_spi_pselinit(struct nrf52_spidev_s *priv,
                               uint32_t offset, nrf52_pinset_t pinset);
static void nrf52_spi_gpioinit(struct nrf52_spidev_s *priv);

#ifdef CONFIG_PM
static int nrf52_spi_deinit(struct nrf52_spidev_s *priv);
static void nrf52_spi_gpiodeinit(struct nrf52_spidev_s *priv);

static int nrf52_spi_pm_prepare(struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate);
static void nrf52_spi_pm_notify(struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM
struct pm_callback_s g_pm_callbacks =
{
  .prepare = nrf52_spi_pm_prepare,
  .notify  = nrf52_spi_pm_notify
};
#endif

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
  .cmddata           = nrf52_spi0cmddata,
#  endif
  .send              = nrf52_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf52_spi_exchange,
#  else
  .sndblock          = nrf52_spi_sndblock,
  .recvblock         = nrf52_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf52_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf52_spi0register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf52_spidev_s g_spi0dev =
{
  .spidev    =
  {
    .ops     = &g_spi0ops,
  },

  .base      = NRF52_SPIM0_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF52_IRQ_SPI_TWI_0,
#endif
  .sck_pin   = BOARD_SPI0_SCK_PIN,
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
  .sndlock           = nrf52_spi_sndblock,
  .recvblock         = nrf52_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf52_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf52_spi1register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf52_spidev_s g_spi1dev =
{
  .spidev    =
  {
    .ops     = &g_spi1ops,
  },

  .base      = NRF52_SPIM1_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF52_IRQ_SPI_TWI_1,
#endif
  .sck_pin   = BOARD_SPI1_SCK_PIN,
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
  .cmddata           = nrf52_spi2cmddata,
#  endif
  .send              = nrf52_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf52_spi_exchange,
#  else
  .sndlock           = nrf52_spi_sndblock,
  .recvblock         = nrf52_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf52_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf52_spi2register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf52_spidev_s g_spi2dev =
{
  .spidev    =
  {
    .ops     = &g_spi2ops,
  },

  .base      = NRF52_SPIM2_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF52_IRQ_SPI2,
#endif
  .sck_pin   = BOARD_SPI2_SCK_PIN,
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
  .cmddata           = nrf52_spi3cmddata,
#  endif
  .send              = nrf52_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf52_spi_exchange,
#  else
  .sndlock           = nrf52_spi_sndblock,
  .recvblock         = nrf52_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf52_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf52_spi3register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf52_spidev_s g_spi3dev =
{
  .spidev    =
  {
    .ops     = &g_spi3ops,
  },

  .base      = NRF52_SPIM3_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF52_IRQ_SPI3,
#endif
  .sck_pin   = BOARD_SPI3_SCK_PIN,
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

static inline void nrf52_spi_putreg(struct nrf52_spidev_s *priv,
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

static inline uint32_t nrf52_spi_getreg(struct nrf52_spidev_s *priv,
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
static int nrf52_spi_isr(int irq, void *context, void *arg)
{
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)arg;

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

static int nrf52_spi_init(struct nrf52_spidev_s *priv)
{
  /* Disable SPI */

  nrf52_spi_putreg(priv, NRF52_SPIM_ENABLE_OFFSET, SPIM_ENABLE_DIS);

  /* Configure SPI pins */

  nrf52_spi_gpioinit(priv);

  /* NOTE: Chip select pin must be configured by board-specific logic */

#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  /* Enable interrupts for RX and TX done */

  nrf52_spi_putreg(priv, NRF52_SPIM_INTENSET_OFFSET, SPIM_INT_END);
#endif

  /* Enable SPI */

  nrf52_spi_putreg(priv, NRF52_SPIM_ENABLE_OFFSET, SPIM_ENABLE_EN);

  return OK;
}

#ifdef CONFIG_PM
/****************************************************************************
 * Name: nrf52_spi_deinit
 *
 * Description:
 *   Configure SPI
 *
 ****************************************************************************/

static int nrf52_spi_deinit(struct nrf52_spidev_s *priv)
{
  /* Disable SPI */

  nrf52_spi_putreg(priv, NRF52_SPIM_ENABLE_OFFSET, SPIM_ENABLE_DIS);

#ifdef CONFIG_ARCH_CHIP_NRF52832
  /* Apply workaround for errata 89 (replace dummy read by barrier to avoid
   * compiler optimizing it away)
   */

  nrf52_spi_putreg(priv, NRF52_SPIM_POWER_OFFSET, 0);
  ARM_DSB();
  ARM_ISB();
  nrf52_spi_putreg(priv, NRF52_SPIM_POWER_OFFSET, 1);
#endif

  /* Unconfigure SPI pins */

  nrf52_spi_gpiodeinit(priv);

  return OK;
}
#endif

/****************************************************************************
 * Name: nrf52_spi_pselinit
 *
 * Description:
 *   Configure PSEL for SPI devices
 *
 ****************************************************************************/

static void nrf52_spi_pselinit(struct nrf52_spidev_s *priv,
                               uint32_t offset, nrf52_pinset_t pinset)
{
  uint32_t regval;
  int pin  = GPIO_PIN_DECODE(pinset);
  int port = GPIO_PORT_DECODE(pinset);

  regval = (pin << SPIM_PSEL_PIN_SHIFT);
  regval |= (port << SPIM_PSEL_PORT_SHIFT);
  nrf52_spi_putreg(priv, offset, regval);
}

/****************************************************************************
 * Name: nrf52_spi_gpioinit
 *
 * Description:
 *   Configure GPIO for SPI pins
 *
 ****************************************************************************/

static void nrf52_spi_gpioinit(struct nrf52_spidev_s *priv)
{
  nrf52_gpio_config(priv->sck_pin);
  nrf52_spi_pselinit(priv, NRF52_SPIM_PSELSCK_OFFSET, priv->sck_pin);

#ifdef CONFIG_NRF52_SPI0_MASTER
  if (priv == &g_spi0dev)
    {
#ifdef BOARD_SPI0_MISO_PIN
      nrf52_gpio_config(BOARD_SPI0_MISO_PIN);
      nrf52_spi_pselinit(priv, NRF52_SPIM_PSELMISO_OFFSET,
                         BOARD_SPI0_MISO_PIN);
#endif
#ifdef BOARD_SPI0_MOSI_PIN
      nrf52_gpio_config(BOARD_SPI0_MOSI_PIN);
      nrf52_spi_pselinit(priv, NRF52_SPIM_PSELMOSI_OFFSET,
                         BOARD_SPI0_MOSI_PIN);
      nrf52_gpio_write(BOARD_SPI0_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF52_SPI1_MASTER
  if (priv == &g_spi1dev)
    {
#ifdef BOARD_SPI1_MISO_PIN
      nrf52_gpio_config(BOARD_SPI1_MISO_PIN);
      nrf52_spi_pselinit(priv, NRF52_SPIM_PSELMISO_OFFSET,
                         BOARD_SPI1_MISO_PIN);
#endif
#ifdef BOARD_SPI1_MOSI_PIN
      nrf52_gpio_config(BOARD_SPI1_MOSI_PIN);
      nrf52_spi_pselinit(priv, NRF52_SPIM_PSELMOSI_OFFSET,
                         BOARD_SPI1_MOSI_PIN);
      nrf52_gpio_write(BOARD_SPI1_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF52_SPI2_MASTER
  if (priv == &g_spi2dev)
    {
#ifdef BOARD_SPI2_MISO_PIN
      nrf52_gpio_config(BOARD_SPI2_MISO_PIN);
      nrf52_spi_pselinit(priv, NRF52_SPIM_PSELMISO_OFFSET,
                         BOARD_SPI2_MISO_PIN);
#endif
#ifdef BOARD_SPI2_MOSI_PIN
      nrf52_gpio_config(BOARD_SPI2_MOSI_PIN);
      nrf52_spi_pselinit(priv, NRF52_SPIM_PSELMOSI_OFFSET,
                         BOARD_SPI2_MOSI_PIN);
      nrf52_gpio_write(BOARD_SPI2_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF52_SPI3_MASTER
  if (priv == &g_spi3dev)
    {
#ifdef BOARD_SPI3_MISO_PIN
      nrf52_gpio_config(BOARD_SPI3_MISO_PIN);
      nrf52_spi_pselinit(priv, NRF52_SPIM_PSELMISO_OFFSET,
                         BOARD_SPI3_MISO_PIN);
#endif
#ifdef BOARD_SPI3_MOSI_PIN
      nrf52_gpio_config(BOARD_SPI3_MOSI_PIN);
      nrf52_spi_pselinit(priv, NRF52_SPIM_PSELMOSI_OFFSET,
                         BOARD_SPI3_MOSI_PIN);
      nrf52_gpio_write(BOARD_SPI3_MOSI_PIN, false);
#endif
    }
#endif
}

#ifdef CONFIG_PM
/****************************************************************************
 * Name: nrf52_spi_gpioinit
 *
 * Description:
 *   Configure GPIO for SPI pins
 *
 ****************************************************************************/

static void nrf52_spi_gpiodeinit(struct nrf52_spidev_s *priv)
{
  nrf52_gpio_unconfig(priv->sck_pin);

#ifdef CONFIG_NRF52_SPI0_MASTER
  if (priv == &g_spi0dev)
    {
#ifdef BOARD_SPI0_MISO_PIN
      nrf52_gpio_unconfig(BOARD_SPI0_MISO_PIN);
#endif
#ifdef BOARD_SPI0_MOSI_PIN
      nrf52_gpio_unconfig(BOARD_SPI0_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF52_SPI1_MASTER
  if (priv == &g_spi1dev)
    {
#ifdef BOARD_SPI1_MISO_PIN
      nrf52_gpio_unconfig(BOARD_SPI1_MISO_PIN);
#endif
#ifdef BOARD_SPI1_MOSI_PIN
      nrf52_gpio_unconfig(BOARD_SPI1_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF52_SPI2_MASTER
  if (priv == &g_spi2dev)
    {
#ifdef BOARD_SPI2_MISO_PIN
      nrf52_gpio_unconfig(BOARD_SPI2_MISO_PIN);
#endif
#ifdef BOARD_SPI2_MOSI_PIN
      nrf52_gpio_unconfig(BOARD_SPI2_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF52_SPI3_MASTER
  if (priv == &g_spi3dev)
    {
#ifdef BOARD_SPI3_MISO_PIN
      nrf52_gpio_unconfig(BOARD_SPI3_MISO_PIN);
#endif
#ifdef BOARD_SPI3_MOSI_PIN
      nrf52_gpio_unconfig(BOARD_SPI3_MOSI_PIN);
#endif
    }
#endif
}
#endif

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

static int nrf52_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;
  int ret = OK;

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

static uint32_t nrf52_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;
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
          spierr("Frequency unsupported %" PRId32 "\n", frequency);
          goto errout;
        }
    }

  /* Write register */

  nrf52_spi_putreg(priv, NRF52_SPIM_FREQUENCY_OFFSET, regval);

  /* Save the frequency setting */

  priv->frequency = frequency;

  spiinfo("Frequency %" PRId32 "\n", frequency);

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

static void nrf52_spi_setmode(struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;
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
              DEBUGPANIC();
              return;
            }
        }

      nrf52_spi_putreg(priv, NRF52_SPIM_CONFIG_OFFSET, regval);

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

static void nrf52_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  if (nbits != 8)
    {
      spierr("ERROR: nbits not supported: %d\n", nbits);
    }
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
static int nrf52_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;
  uint32_t regval;

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

static uint32_t nrf52_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  uint32_t ret = 0;

  /* Exchange one word on SPI */

  nrf52_spi_exchange(dev, &wd, &ret, 1);

  return ret;
}

#ifdef CONFIG_NRF52_SPI_MASTER_WORKAROUND_1BYTE_TRANSFER

/****************************************************************************
 * Name: n4f52_spi_1b_workaround
 *
 * Description:
 *   Workaround to fix SPI Master 1 byte transfer for NRF52832.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   enable - Enable/disable workaround
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf52_spi_1b_workaround(struct spi_dev_s *dev, bool enable)
{
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;
  uint32_t pin  = 0;
  uint32_t port = 0;

  pin = (priv->sck_pin & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  port = (priv->sck_pin & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  if (enable == true)
    {
      /* Create an event when SCK toggles */

      putreg32((GPIOTE_CONFIG_MODE_EV |
                pin << GPIOTE_CONFIG_PSEL_SHIFT |
                port << GPIOTE_CONFIG_PORT_SHIFT |
                GPIOTE_CONFIG_POL_TG),
               NRF52_GPIOTE_CONFIG(SPI_1B_WORKAROUND_GPIOTE_CHAN));

      /* Stop the SPIM instance when SCK toggles */

      nrf52_ppi_set_event_ep(SPI_1B_WORKAROUND_PPI_CHAN,
                             NRF52_GPIOTE_EVENTS_IN(
                               SPI_1B_WORKAROUND_GPIOTE_CHAN));

      nrf52_ppi_set_task_ep(SPI_1B_WORKAROUND_PPI_CHAN,
                            priv->base + NRF52_SPIM_TASK_STOP_OFFSET);

      /* Enable PPI channel */

      nrf52_ppi_channel_enable(SPI_1B_WORKAROUND_PPI_CHAN, true);
    }
  else
    {
      /* Disable event */

      putreg32(0, NRF52_GPIOTE_CONFIG(SPI_1B_WORKAROUND_GPIOTE_CHAN));

      nrf52_ppi_set_event_ep(SPI_1B_WORKAROUND_PPI_CHAN, 0);
      nrf52_ppi_set_task_ep(SPI_1B_WORKAROUND_PPI_CHAN, 0);

      /* Disable PPI channel */

      nrf52_ppi_channel_enable(SPI_1B_WORKAROUND_PPI_CHAN, false);
    }
}
#endif

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
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nrf52_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords)
{
  struct nrf52_spidev_s *priv = (struct nrf52_spidev_s *)dev;
  uint32_t regval = 0;
  size_t nwords_left = nwords;

#ifdef CONFIG_NRF52_SPI_MASTER_WORKAROUND_1BYTE_TRANSFER
  if (nwords <= 1)
    {
      nrf52_spi_1b_workaround(dev, true);
    }
#endif

  if (rxbuffer != NULL)
    {
      /* Write RXD data pointer */

      regval = (uint32_t)rxbuffer;
      nrf52_spi_putreg(priv, NRF52_SPIM_RXDPTR_OFFSET, regval);
    }
  else
    {
      nrf52_spi_putreg(priv, NRF52_SPIM_RXDMAXCNT_OFFSET, 0);
    }

  if (txbuffer != NULL)
    {
      /* Write TXD data pointer */

      regval = (uint32_t)txbuffer;
      nrf52_spi_putreg(priv, NRF52_SPIM_TXDPTR_OFFSET, regval);
    }
  else
    {
      nrf52_spi_putreg(priv, NRF52_SPIM_TXDMAXCNT_OFFSET, 0);
    }

  /* If more than 255 bytes, enable list mode to send data
   * in batches
   */

  if (nwords > 0xff)
    {
      if (rxbuffer != NULL)
        {
          nrf52_spi_putreg(priv, NRF52_SPIM_RXDLIST_OFFSET, 1);
        }

      if (txbuffer != NULL)
        {
          nrf52_spi_putreg(priv, NRF52_SPIM_TXDLIST_OFFSET, 1);
        }
    }

  while (nwords_left > 0)
    {
      size_t transfer_size = (nwords_left > 255 ? 255 : nwords_left);

      if (rxbuffer != NULL)
        {
          /* Write number of bytes in RXD buffer */

          nrf52_spi_putreg(priv, NRF52_SPIM_RXDMAXCNT_OFFSET, transfer_size);
        }

      if (txbuffer != NULL)
        {
          /* Write number of bytes in TXD buffer */

          nrf52_spi_putreg(priv, NRF52_SPIM_TXDMAXCNT_OFFSET, transfer_size);
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

      nxsem_wait_uninterruptible(&priv->sem_isr);
#endif

      if (nrf52_spi_getreg(priv, NRF52_SPIM_TXDAMOUNT_OFFSET) !=
          transfer_size)
        {
          spierr("Incomplete transfer wrote %" PRId32 " expected %zu\n",
                 regval, nwords);
        }

      /* SPI stop */

      nrf52_spi_putreg(priv, NRF52_SPIM_TASK_STOP_OFFSET, SPIM_TASKS_STOP);

      /* Wait for STOP event */

      while (nrf52_spi_getreg(priv, NRF52_SPIM_EVENTS_STOPPED_OFFSET) != 1);

      /* Clear event */

      nrf52_spi_putreg(priv, NRF52_SPIM_EVENTS_STOPPED_OFFSET, 0);

      nwords_left -= transfer_size;
    }

  /* Clear RX/TX DMA after transfer */

  nrf52_spi_putreg(priv, NRF52_SPIM_RXDPTR_OFFSET, 0);
  nrf52_spi_putreg(priv, NRF52_SPIM_RXDMAXCNT_OFFSET, 0);
  nrf52_spi_putreg(priv, NRF52_SPIM_TXDPTR_OFFSET, 0);
  nrf52_spi_putreg(priv, NRF52_SPIM_TXDMAXCNT_OFFSET, 0);

  /* Clear list mode */

  if (nwords > 0xff)
    {
      nrf52_spi_putreg(priv, NRF52_SPIM_RXDLIST_OFFSET, 0);
      nrf52_spi_putreg(priv, NRF52_SPIM_TXDLIST_OFFSET, 0);
    }

#ifdef CONFIG_NRF52_SPI_MASTER_WORKAROUND_1BYTE_TRANSFER
  if (nwords <= 1)
    {
      nrf52_spi_1b_workaround(dev, false);
    }
#endif
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

static void nrf52_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer,
                               size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%zu\n", txbuffer, nwords);
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

static void nrf52_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer,
                                size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%zu\n", rxbuffer, nwords);
  return nrf52_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif /* CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Name: nrf52_spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   -ENOSYS  - Trigger not fired due to lack of DMA or low level support
 *   -EIO     - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int nrf52_spi_trigger(struct spi_dev_s *dev)
{
  return -ENOSYS;
}
#endif

#ifdef CONFIG_PM
/****************************************************************************
 * Name: nrf52_spi_pm_prepare
 ****************************************************************************/

static int nrf52_spi_pm_prepare(struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate)
{
  if (pmstate == PM_STANDBY || pmstate == PM_SLEEP)
    {
      bool active = false;

#ifdef CONFIG_NRF52_SPI0_MASTER
      active |= nrf52_spi_getreg(&g_spi0dev, SPIM_EVENTS_STARTED);
#endif
#ifdef CONFIG_NRF52_SPI1_MASTER
      active |= nrf52_spi_getreg(&g_spi1dev, SPIM_EVENTS_STARTED);
#endif
#ifdef CONFIG_NRF52_SPI2_MASTER
      active |= nrf52_spi_getreg(&g_spi2dev, SPIM_EVENTS_STARTED);
#endif
#ifdef CONFIG_NRF52_SPI3_MASTER
      active |= nrf52_spi_getreg(&g_spi3dev, SPIM_EVENTS_STARTED);
#endif

      if (active)
        {
          /* SPI is being used, cannot disable */

          return -1;
        }
      else
        {
          /* SPI is inactive, can go to sleep */

          return 0;
        }
    }
  else
    {
      /* We can always go to any other state */

      return 0;
    }
}

/****************************************************************************
 * Name: nrf52_spi_pm_notify
 ****************************************************************************/

static void nrf52_spi_pm_notify(struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate)
{
  if (pmstate == PM_SLEEP || pmstate == PM_STANDBY)
    {
      /* Deinit SPI peripheral on each initialized device */

#ifdef CONFIG_NRF52_SPI0_MASTER
      if (g_spi0dev.initialized)
        {
          nrf52_spi_deinit(&g_spi0dev);
        }
#endif

#ifdef CONFIG_NRF52_SPI1_MASTER
      if (g_spi1dev.initialized)
        {
          nrf52_spi_deinit(&g_spi1dev);
        }
#endif

#ifdef CONFIG_NRF52_SPI2_MASTER
      if (g_spi2dev.initialized)
        {
          nrf52_spi_deinit(&g_spi2dev);
        }
#endif

#ifdef CONFIG_NRF52_SPI3_MASTER
      if (g_spi3dev.initialized)
        {
          nrf52_spi_deinit(&g_spi3dev);
        }
#endif
    }
  else
    {
      /* Reinit SPI peripheral on each initialized device */

#ifdef CONFIG_NRF52_SPI0_MASTER
      if (g_spi0dev.initialized)
        {
          nrf52_spi_init(&g_spi0dev);
        }
#endif

#ifdef CONFIG_NRF52_SPI1_MASTER
      if (g_spi1dev.initialized)
        {
          nrf52_spi_init(&g_spi1dev);
        }
#endif

#ifdef CONFIG_NRF52_SPI2_MASTER
      if (g_spi2dev.initialized)
        {
          nrf52_spi_init(&g_spi2dev);
        }
#endif

#ifdef CONFIG_NRF52_SPI3_MASTER
      if (g_spi3dev.initialized)
        {
          nrf52_spi_init(&g_spi3dev);
        }
#endif
    }
}
#endif

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

struct spi_dev_s *nrf52_spibus_initialize(int port)
{
  struct nrf52_spidev_s *priv = NULL;

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

  /* Mark device as initialized */

  priv->initialized = true;

#ifdef CONFIG_NRF52_SPI_MASTER_INTERRUPTS
  /* Attach SPI interrupt */

  irq_attach(priv->irq, nrf52_spi_isr, priv);
  up_enable_irq(priv->irq);
#endif

errout:
  return (struct spi_dev_s *)priv;
}
