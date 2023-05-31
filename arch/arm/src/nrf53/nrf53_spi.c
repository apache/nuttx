/****************************************************************************
 * arch/arm/src/nrf53/nrf53_spi.c
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

#include "nrf53_gpio.h"
#include "nrf53_spi.h"

#include "hardware/nrf53_spi.h"
#include "hardware/nrf53_utils.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf53_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         base;       /* Base address of SPI register */
#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
  uint32_t         irq;        /* SPI IRQ number */
#endif
  nrf53_pinset_t   sck_pin;    /* SCK pin configuration */
  uint32_t         frequency;  /* Requested clock frequency */
  uint8_t          mode;       /* Mode 0,1,2,3 */

  mutex_t          lock;       /* Held while chip is selected for mutual
                                * exclusion
                                */
#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
  sem_t            sem_isr;    /* Interrupt wait semaphore */
#endif
  bool             initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void nrf53_spi_putreg(struct nrf53_spidev_s *priv,
                                    uint32_t offset,
                                    uint32_t value);
static inline uint32_t nrf53_spi_getreg(struct nrf53_spidev_s *priv,
                                        uint32_t offset);

/* SPI methods */

static int nrf53_spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t nrf53_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
static void nrf53_spi_setmode(struct spi_dev_s *priv,
                              enum spi_mode_e mode);
static void nrf53_spi_setbits(struct spi_dev_s *priv, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int nrf53_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint32_t nrf53_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void nrf53_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void nrf53_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer,
                               size_t nwords);
static void nrf53_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer,
                                size_t nwords);
#endif

#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
static int nrf53_spi_isr(int irq, void *context, void *arg);
#endif

/* Initialization */

static int nrf53_spi_init(struct nrf53_spidev_s *priv);
static void nrf53_spi_pselinit(struct nrf53_spidev_s *priv,
                               uint32_t offset, nrf53_pinset_t pinset);
static void nrf53_spi_gpioinit(struct nrf53_spidev_s *priv);

#ifdef CONFIG_PM
static int nrf53_spi_deinit(struct nrf53_spidev_s *priv);
static void nrf53_spi_gpiodeinit(struct nrf53_spidev_s *priv);

static int nrf53_spi_pm_prepare(struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate);
static void nrf53_spi_pm_notify(struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM
struct pm_callback_s g_pm_callbacks =
{
  .prepare = nrf53_spi_pm_prepare,
  .notify  = nrf53_spi_pm_notify
};
#endif

/* SPI0 */

#ifdef CONFIG_NRF53_SPI0_MASTER
static const struct spi_ops_s g_spi0ops =
{
  .lock              = nrf53_spi_lock,
  .select            = nrf53_spi0select,
  .setfrequency      = nrf53_spi_setfrequency,
  .setmode           = nrf53_spi_setmode,
  .setbits           = nrf53_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf53_spi_hwfeatures,
#  endif
  .status            = nrf53_spi0status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf53_spi0cmddata,
#  endif
  .send              = nrf53_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf53_spi_exchange,
#  else
  .sndblock          = nrf53_spi_sndblock,
  .recvblock         = nrf53_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf53_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf53_spi0register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf53_spidev_s g_spi0dev =
{
  .spidev    =
  {
    .ops     = &g_spi0ops,
  },

  .base      = NRF53_SPIM0_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF53_IRQ_SERIAL0,
#endif
  .sck_pin   = BOARD_SPI0_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI1 */

#ifdef CONFIG_NRF53_SPI1_MASTER
static const struct spi_ops_s g_spi1ops =
{
  .lock              = nrf53_spi_lock,
  .select            = nrf53_spi1select,
  .setfrequency      = nrf53_spi_setfrequency,
  .setmode           = nrf53_spi_setmode,
  .setbits           = nrf53_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf53_spi_hwfeatures,
#  endif
  .status            = nrf53_spi1status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf53_spi1cmddata,
#  endif
  .send              = nrf53_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf53_spi_exchange,
#  else
  .sndlock           = nrf53_spi_sndblock,
  .recvblock         = nrf53_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf53_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf53_spi1register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf53_spidev_s g_spi1dev =
{
  .spidev    =
  {
    .ops     = &g_spi1ops,
  },

  .base      = NRF53_SPIM1_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF53_IRQ_SERIAL1,
#endif
  .sck_pin   = BOARD_SPI1_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI2 */

#ifdef CONFIG_NRF53_SPI2_MASTER
static const struct spi_ops_s g_spi2ops =
{
  .lock              = nrf53_spi_lock,
  .select            = nrf53_spi2select,
  .setfrequency      = nrf53_spi_setfrequency,
  .setmode           = nrf53_spi_setmode,
  .setbits           = nrf53_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf53_spi_hwfeatures,
#  endif
  .status            = nrf53_spi2status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf53_spi2cmddata,
#  endif
  .send              = nrf53_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf53_spi_exchange,
#  else
  .sndlock           = nrf53_spi_sndblock,
  .recvblock         = nrf53_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf53_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf53_spi2register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf53_spidev_s g_spi2dev =
{
  .spidev    =
  {
    .ops     = &g_spi2ops,
  },

  .base      = NRF53_SPIM2_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF53_IRQ_SERIAL2,
#endif
  .sck_pin   = BOARD_SPI2_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI3 */

#ifdef CONFIG_NRF53_SPI3_MASTER
static const struct spi_ops_s g_spi3ops =
{
  .lock              = nrf53_spi_lock,
  .select            = nrf53_spi3select,
  .setfrequency      = nrf53_spi_setfrequency,
  .setmode           = nrf53_spi_setmode,
  .setbits           = nrf53_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf53_spi_hwfeatures,
#  endif
  .status            = nrf53_spi3status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf53_spi3cmddata,
#  endif
  .send              = nrf53_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf53_spi_exchange,
#  else
  .sndlock           = nrf53_spi_sndblock,
  .recvblock         = nrf53_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf53_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf53_spi3register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf53_spidev_s g_spi3dev =
{
  .spidev    =
  {
    .ops     = &g_spi3ops,
  },

  .base      = NRF53_SPIM3_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF53_IRQ_SERIAL3,
#endif
  .sck_pin   = BOARD_SPI3_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/* SPI4 */

#ifdef CONFIG_NRF53_SPI4_MASTER
static const struct spi_ops_s g_spi4ops =
{
  .lock              = nrf53_spi_lock,
  .select            = nrf53_spi4select,
  .setfrequency      = nrf53_spi_setfrequency,
  .setmode           = nrf53_spi_setmode,
  .setbits           = nrf53_spi_setbits,
#  ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = nrf53_spi_hwfeatures,
#  endif
  .status            = nrf53_spi4status,
#  ifdef CONFIG_SPI_CMDDATA
  .cmddata           = nrf53_spi4cmddata,
#  endif
  .send              = nrf53_spi_send,
#  ifdef CONFIG_SPI_EXCHANGE
  .exchange          = nrf53_spi_exchange,
#  else
  .sndlock           = nrf53_spi_sndblock,
  .recvblock         = nrf53_spi_recvblock,
#  endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = nrf53_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = nrf53_spi4register,  /* Provided externally */
#else
  .registercallback  = NULL,                /* Not implemented */
#endif
};

static struct nrf53_spidev_s g_spi4dev =
{
  .spidev    =
  {
    .ops     = &g_spi4ops,
  },

  .base      = NRF53_SPIM4_BASE,
  .lock      = NXMUTEX_INITIALIZER,
#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
  .sem_isr   = SEM_INITIALIZER(0),
  .irq       = NRF53_IRQ_SPI4,
#endif
  .sck_pin   = BOARD_SPI4_SCK_PIN,
  .frequency = 0,
  .mode      = 0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_spi_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void nrf53_spi_putreg(struct nrf53_spidev_s *priv,
                                    uint32_t offset,
                                    uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: nrf53_spi_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t nrf53_spi_getreg(struct nrf53_spidev_s *priv,
                                        uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: nrf53_spi_isr
 *
 * Description:
 *   Common SPI interrupt service routine
 *
 ****************************************************************************/

#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
static int nrf53_spi_isr(int irq, void *context, void *arg)
{
  struct nrf53_spidev_s *priv = (struct nrf53_spidev_s *)arg;

  /* Get interrupt event */

  if (nrf53_spi_getreg(priv, NRF53_SPIM_EVENTS_END_OFFSET) == 1)
    {
      /* Transfer is complete */

      nxsem_post(&priv->sem_isr);

      /* Clear event */

      nrf53_spi_putreg(priv, NRF53_SPIM_EVENTS_END_OFFSET, 0);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: nrf53_spi_init
 *
 * Description:
 *   Configure SPI
 *
 ****************************************************************************/

static int nrf53_spi_init(struct nrf53_spidev_s *priv)
{
  /* Disable SPI */

  nrf53_spi_putreg(priv, NRF53_SPIM_ENABLE_OFFSET, SPIM_ENABLE_DIS);

  /* Configure SPI pins */

  nrf53_spi_gpioinit(priv);

  /* NOTE: Chip select pin must be configured by board-specific logic */

#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
  /* Enable interrupts for RX and TX done */

  nrf53_spi_putreg(priv, NRF53_SPIM_INTENSET_OFFSET, SPIM_INT_END);
#endif

  /* Enable SPI */

  nrf53_spi_putreg(priv, NRF53_SPIM_ENABLE_OFFSET, SPIM_ENABLE_EN);

  return OK;
}

#ifdef CONFIG_PM
/****************************************************************************
 * Name: nrf53_spi_deinit
 *
 * Description:
 *   Configure SPI
 *
 ****************************************************************************/

static int nrf53_spi_deinit(struct nrf53_spidev_s *priv)
{
  /* Disable SPI */

  nrf53_spi_putreg(priv, NRF53_SPIM_ENABLE_OFFSET, SPIM_ENABLE_DIS);

  /* Unconfigure SPI pins */

  nrf53_spi_gpiodeinit(priv);

  return OK;
}
#endif

/****************************************************************************
 * Name: nrf53_spi_pselinit
 *
 * Description:
 *   Configure PSEL for SPI devices
 *
 ****************************************************************************/

static void nrf53_spi_pselinit(struct nrf53_spidev_s *priv,
                               uint32_t offset, nrf53_pinset_t pinset)
{
  uint32_t regval;
  int pin  = GPIO_PIN_DECODE(pinset);
  int port = GPIO_PORT_DECODE(pinset);

  regval = (pin << SPIM_PSEL_PIN_SHIFT);
  regval |= (port << SPIM_PSEL_PORT_SHIFT);
  nrf53_spi_putreg(priv, offset, regval);
}

/****************************************************************************
 * Name: nrf53_spi_gpioinit
 *
 * Description:
 *   Configure GPIO for SPI pins
 *
 ****************************************************************************/

static void nrf53_spi_gpioinit(struct nrf53_spidev_s *priv)
{
  nrf53_gpio_config(priv->sck_pin);
  nrf53_spi_pselinit(priv, NRF53_SPIM_PSELSCK_OFFSET, priv->sck_pin);

#ifdef CONFIG_NRF53_SPI0_MASTER
  if (priv == &g_spi0dev)
    {
#ifdef BOARD_SPI0_MISO_PIN
      nrf53_gpio_config(BOARD_SPI0_MISO_PIN);
      nrf53_spi_pselinit(priv, NRF53_SPIM_PSELMISO_OFFSET,
                         BOARD_SPI0_MISO_PIN);
#endif
#ifdef BOARD_SPI0_MOSI_PIN
      nrf53_gpio_config(BOARD_SPI0_MOSI_PIN);
      nrf53_spi_pselinit(priv, NRF53_SPIM_PSELMOSI_OFFSET,
                         BOARD_SPI0_MOSI_PIN);
      nrf53_gpio_write(BOARD_SPI0_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF53_SPI1_MASTER
  if (priv == &g_spi1dev)
    {
#ifdef BOARD_SPI1_MISO_PIN
      nrf53_gpio_config(BOARD_SPI1_MISO_PIN);
      nrf53_spi_pselinit(priv, NRF53_SPIM_PSELMISO_OFFSET,
                         BOARD_SPI1_MISO_PIN);
#endif
#ifdef BOARD_SPI1_MOSI_PIN
      nrf53_gpio_config(BOARD_SPI1_MOSI_PIN);
      nrf53_spi_pselinit(priv, NRF53_SPIM_PSELMOSI_OFFSET,
                         BOARD_SPI1_MOSI_PIN);
      nrf53_gpio_write(BOARD_SPI1_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF53_SPI2_MASTER
  if (priv == &g_spi2dev)
    {
#ifdef BOARD_SPI2_MISO_PIN
      nrf53_gpio_config(BOARD_SPI2_MISO_PIN);
      nrf53_spi_pselinit(priv, NRF53_SPIM_PSELMISO_OFFSET,
                         BOARD_SPI2_MISO_PIN);
#endif
#ifdef BOARD_SPI2_MOSI_PIN
      nrf53_gpio_config(BOARD_SPI2_MOSI_PIN);
      nrf53_spi_pselinit(priv, NRF53_SPIM_PSELMOSI_OFFSET,
                         BOARD_SPI2_MOSI_PIN);
      nrf53_gpio_write(BOARD_SPI2_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF53_SPI3_MASTER
  if (priv == &g_spi3dev)
    {
#ifdef BOARD_SPI3_MISO_PIN
      nrf53_gpio_config(BOARD_SPI3_MISO_PIN);
      nrf53_spi_pselinit(priv, NRF53_SPIM_PSELMISO_OFFSET,
                         BOARD_SPI3_MISO_PIN);
#endif
#ifdef BOARD_SPI3_MOSI_PIN
      nrf53_gpio_config(BOARD_SPI3_MOSI_PIN);
      nrf53_spi_pselinit(priv, NRF53_SPIM_PSELMOSI_OFFSET,
                         BOARD_SPI3_MOSI_PIN);
      nrf53_gpio_write(BOARD_SPI3_MOSI_PIN, false);
#endif
    }
#endif

#ifdef CONFIG_NRF53_SPI4_MASTER
  if (priv == &g_spi4dev)
    {
#ifdef BOARD_SPI4_MISO_PIN
      nrf53_gpio_config(BOARD_SPI4_MISO_PIN);
      nrf53_spi_pselinit(priv, NRF53_SPIM_PSELMISO_OFFSET,
                         BOARD_SPI4_MISO_PIN);
#endif
#ifdef BOARD_SPI4_MOSI_PIN
      nrf53_gpio_config(BOARD_SPI4_MOSI_PIN);
      nrf53_spi_pselinit(priv, NRF53_SPIM_PSELMOSI_OFFSET,
                         BOARD_SPI4_MOSI_PIN);
      nrf53_gpio_write(BOARD_SPI4_MOSI_PIN, false);
#endif
    }
#endif
}

#ifdef CONFIG_PM
/****************************************************************************
 * Name: nrf53_spi_gpioinit
 *
 * Description:
 *   Configure GPIO for SPI pins
 *
 ****************************************************************************/

static void nrf53_spi_gpiodeinit(struct nrf53_spidev_s *priv)
{
  nrf53_gpio_unconfig(priv->sck_pin);

#ifdef CONFIG_NRF53_SPI0_MASTER
  if (priv == &g_spi0dev)
    {
#ifdef BOARD_SPI0_MISO_PIN
      nrf53_gpio_unconfig(BOARD_SPI0_MISO_PIN);
#endif
#ifdef BOARD_SPI0_MOSI_PIN
      nrf53_gpio_unconfig(BOARD_SPI0_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF53_SPI1_MASTER
  if (priv == &g_spi1dev)
    {
#ifdef BOARD_SPI1_MISO_PIN
      nrf53_gpio_unconfig(BOARD_SPI1_MISO_PIN);
#endif
#ifdef BOARD_SPI1_MOSI_PIN
      nrf53_gpio_unconfig(BOARD_SPI1_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF53_SPI2_MASTER
  if (priv == &g_spi2dev)
    {
#ifdef BOARD_SPI2_MISO_PIN
      nrf53_gpio_unconfig(BOARD_SPI2_MISO_PIN);
#endif
#ifdef BOARD_SPI2_MOSI_PIN
      nrf53_gpio_unconfig(BOARD_SPI2_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF53_SPI3_MASTER
  if (priv == &g_spi3dev)
    {
#ifdef BOARD_SPI3_MISO_PIN
      nrf53_gpio_unconfig(BOARD_SPI3_MISO_PIN);
#endif
#ifdef BOARD_SPI3_MOSI_PIN
      nrf53_gpio_unconfig(BOARD_SPI3_MOSI_PIN);
#endif
    }
#endif

#ifdef CONFIG_NRF53_SPI4_MASTER
  if (priv == &g_spi4dev)
    {
#ifdef BOARD_SPI4_MISO_PIN
      nrf53_gpio_unconfig(BOARD_SPI4_MISO_PIN);
#endif
#ifdef BOARD_SPI4_MOSI_PIN
      nrf53_gpio_unconfig(BOARD_SPI4_MOSI_PIN);
#endif
    }
#endif
}
#endif

/****************************************************************************
 * Name: nrf53_spi_lock
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

static int nrf53_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct nrf53_spidev_s *priv = (struct nrf53_spidev_s *)dev;
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
 * Name: nrf53_spi_setfrequency
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

static uint32_t nrf53_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct nrf53_spidev_s *priv = (struct nrf53_spidev_s *)dev;
  uint32_t regval = 0;

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency */

      return priv->frequency;
    }

  /* Frequency > 8MHz available only for SPIM4 */

  if (frequency > 8000000 && priv->base != NRF53_SPIM4_BASE)
    {
      frequency = 8000000;
      spiwarn("Reduce freq to %" PRId32 "\n", frequency);
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

      case 16000000:
        {
          regval = SPIM_FREQUENCY_16MBPS;
          break;
        }

      case 32000000:
        {
          regval = SPIM_FREQUENCY_32MBPS;
          break;
        }

      default:
        {
          spierr("Frequency unsupported %" PRId32 "\n", frequency);
          goto errout;
        }
    }

  /* Write register */

  nrf53_spi_putreg(priv, NRF53_SPIM_FREQUENCY_OFFSET, regval);

  /* Save the frequency setting */

  priv->frequency = frequency;

  spiinfo("Frequency %" PRId32 "\n", frequency);

errout:
  return priv->frequency;
}

/****************************************************************************
 * Name: nrf53_spi_setmode
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

static void nrf53_spi_setmode(struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  struct nrf53_spidev_s *priv = (struct nrf53_spidev_s *)dev;
  uint32_t regval = 0;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      regval = nrf53_spi_getreg(priv, NRF53_SPIM_CONFIG_OFFSET);
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

      nrf53_spi_putreg(priv, NRF53_SPIM_CONFIG_OFFSET, regval);

      /* According to manual we have to set SCK pin output
       * value the same as CPOL value
       */

      if (mode == SPIDEV_MODE2 || mode == SPIDEV_MODE3)
        {
          nrf53_gpio_write(priv->sck_pin, true);
        }
      else
        {
          nrf53_gpio_write(priv->sck_pin, false);
        }

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: nrf53_spi_setbits
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

static void nrf53_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  if (nbits != 8)
    {
      spierr("ERROR: nbits not supported: %d\n", nbits);
    }
}

/****************************************************************************
 * Name: nrf53_spi_hwfeatures
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
static int nrf53_spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct nrf53_spidev_s *priv = (struct nrf53_spidev_s *)dev;
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

  regval = nrf53_spi_getreg(priv, NRF53_SPIM_CONFIG_OFFSET);
  regval &= ~clrbits;
  regval |= setbits;
  nrf53_spi_putreg(priv, NRF53_SPIM_CONFIG_OFFSET, regval);

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

static uint32_t nrf53_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  uint32_t ret = 0;

  /* Exchange one word on SPI */

  nrf53_spi_exchange(dev, &wd, &ret, 1);

  return ret;
}

/****************************************************************************
 * Name: nrf53_spi_exchange
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

static void nrf53_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer,
                               void *rxbuffer, size_t nwords)
{
  struct nrf53_spidev_s *priv = (struct nrf53_spidev_s *)dev;
  uint32_t regval = 0;
  size_t nwords_left = nwords;

  if (rxbuffer != NULL)
    {
      /* Write RXD data pointer */

      regval = (uint32_t)rxbuffer;
      DEBUGASSERT(nrf53_easydma_valid(regval));
      nrf53_spi_putreg(priv, NRF53_SPIM_RXDPTR_OFFSET, regval);
    }
  else
    {
      nrf53_spi_putreg(priv, NRF53_SPIM_RXDMAXCNT_OFFSET, 0);
    }

  if (txbuffer != NULL)
    {
      /* Write TXD data pointer */

      regval = (uint32_t)txbuffer;
      DEBUGASSERT(nrf53_easydma_valid(regval));
      nrf53_spi_putreg(priv, NRF53_SPIM_TXDPTR_OFFSET, regval);
    }
  else
    {
      nrf53_spi_putreg(priv, NRF53_SPIM_TXDMAXCNT_OFFSET, 0);
    }

  /* If more than 255 bytes, enable list mode to send data
   * in batches
   */

  if (nwords > 0xff)
    {
      if (rxbuffer != NULL)
        {
          nrf53_spi_putreg(priv, NRF53_SPIM_RXDLIST_OFFSET, 1);
        }

      if (txbuffer != NULL)
        {
          nrf53_spi_putreg(priv, NRF53_SPIM_TXDLIST_OFFSET, 1);
        }
    }

  while (nwords_left > 0)
    {
      size_t transfer_size = (nwords_left > 255 ? 255 : nwords_left);

      if (rxbuffer != NULL)
        {
          /* Write number of bytes in RXD buffer */

          nrf53_spi_putreg(priv, NRF53_SPIM_RXDMAXCNT_OFFSET, transfer_size);
        }

      if (txbuffer != NULL)
        {
          /* Write number of bytes in TXD buffer */

          nrf53_spi_putreg(priv, NRF53_SPIM_TXDMAXCNT_OFFSET, transfer_size);
        }

      /* SPI start */

      nrf53_spi_putreg(priv, NRF53_SPIM_TASK_START_OFFSET, SPIM_TASKS_START);

#ifndef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
      /* Wait for RX done and TX done */

      while (nrf53_spi_getreg(priv, NRF53_SPIM_EVENTS_END_OFFSET) != 1);

      /* Clear event */

      nrf53_spi_putreg(priv, NRF53_SPIM_EVENTS_END_OFFSET, 0);
#else
      /* Wait for transfer complete */

      nxsem_wait_uninterruptible(&priv->sem_isr);
#endif

      if (nrf53_spi_getreg(priv, NRF53_SPIM_TXDAMOUNT_OFFSET) !=
          transfer_size)
        {
          spierr("Incomplete transfer wrote %" PRId32 " expected %zu\n",
                 regval, nwords);
        }

      /* SPI stop */

      nrf53_spi_putreg(priv, NRF53_SPIM_TASK_STOP_OFFSET, SPIM_TASKS_STOP);

      /* Wait for STOP event */

      while (nrf53_spi_getreg(priv, NRF53_SPIM_EVENTS_STOPPED_OFFSET) != 1);

      /* Clear event */

      nrf53_spi_putreg(priv, NRF53_SPIM_EVENTS_STOPPED_OFFSET, 0);

      nwords_left -= transfer_size;
    }

  /* Clear RX/TX DMA after transfer */

  nrf53_spi_putreg(priv, NRF53_SPIM_RXDPTR_OFFSET, 0);
  nrf53_spi_putreg(priv, NRF53_SPIM_RXDMAXCNT_OFFSET, 0);
  nrf53_spi_putreg(priv, NRF53_SPIM_TXDPTR_OFFSET, 0);
  nrf53_spi_putreg(priv, NRF53_SPIM_TXDMAXCNT_OFFSET, 0);

  /* Clear list mode */

  if (nwords > 0xff)
    {
      nrf53_spi_putreg(priv, NRF53_SPIM_RXDLIST_OFFSET, 0);
      nrf53_spi_putreg(priv, NRF53_SPIM_TXDLIST_OFFSET, 0);
    }
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: nrf53_spi_sndblock
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

static void nrf53_spi_sndblock(struct spi_dev_s *dev,
                               const void *txbuffer,
                               size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%zu\n", txbuffer, nwords);
  return nrf53_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: nrf53_spi_recvblock
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

static void nrf53_spi_recvblock(struct spi_dev_s *dev,
                                void *rxbuffer,
                                size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%zu\n", rxbuffer, nwords);
  return nrf53_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif /* CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Name: nrf53_spi_trigger
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
static int nrf53_spi_trigger(struct spi_dev_s *dev)
{
  return -ENOSYS;
}
#endif

#ifdef CONFIG_PM
/****************************************************************************
 * Name: nrf53_spi_pm_prepare
 ****************************************************************************/

static int nrf53_spi_pm_prepare(struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate)
{
  if (pmstate == PM_STANDBY || pmstate == PM_SLEEP)
    {
      bool active = false;

#ifdef CONFIG_NRF53_SPI0_MASTER
      active |= nrf53_spi_getreg(&g_spi0dev, SPIM_EVENTS_STARTED);
#endif
#ifdef CONFIG_NRF53_SPI1_MASTER
      active |= nrf53_spi_getreg(&g_spi1dev, SPIM_EVENTS_STARTED);
#endif
#ifdef CONFIG_NRF53_SPI2_MASTER
      active |= nrf53_spi_getreg(&g_spi2dev, SPIM_EVENTS_STARTED);
#endif
#ifdef CONFIG_NRF53_SPI3_MASTER
      active |= nrf53_spi_getreg(&g_spi3dev, SPIM_EVENTS_STARTED);
#endif
#ifdef CONFIG_NRF53_SPI4_MASTER
      active |= nrf53_spi_getreg(&g_spi4dev, SPIM_EVENTS_STARTED);
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
 * Name: nrf53_spi_pm_notify
 ****************************************************************************/

static void nrf53_spi_pm_notify(struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate)
{
  if (pmstate == PM_SLEEP || pmstate == PM_STANDBY)
    {
      /* Deinit SPI peripheral on each initialized device */

#ifdef CONFIG_NRF53_SPI0_MASTER
      if (g_spi0dev.initialized)
        {
          nrf53_spi_deinit(&g_spi0dev);
        }
#endif

#ifdef CONFIG_NRF53_SPI1_MASTER
      if (g_spi1dev.initialized)
        {
          nrf53_spi_deinit(&g_spi1dev);
        }
#endif

#ifdef CONFIG_NRF53_SPI2_MASTER
      if (g_spi2dev.initialized)
        {
          nrf53_spi_deinit(&g_spi2dev);
        }
#endif

#ifdef CONFIG_NRF53_SPI3_MASTER
      if (g_spi3dev.initialized)
        {
          nrf53_spi_deinit(&g_spi3dev);
        }
#endif

#ifdef CONFIG_NRF53_SPI4_MASTER
      if (g_spi4dev.initialized)
        {
          nrf53_spi_deinit(&g_spi4dev);
        }
#endif
    }
  else
    {
      /* Reinit SPI peripheral on each initialized device */

#ifdef CONFIG_NRF53_SPI0_MASTER
      if (g_spi0dev.initialized)
        {
          nrf53_spi_init(&g_spi0dev);
        }
#endif

#ifdef CONFIG_NRF53_SPI1_MASTER
      if (g_spi1dev.initialized)
        {
          nrf53_spi_init(&g_spi1dev);
        }
#endif

#ifdef CONFIG_NRF53_SPI2_MASTER
      if (g_spi2dev.initialized)
        {
          nrf53_spi_init(&g_spi2dev);
        }
#endif

#ifdef CONFIG_NRF53_SPI3_MASTER
      if (g_spi3dev.initialized)
        {
          nrf53_spi_init(&g_spi3dev);
        }
#endif

#ifdef CONFIG_NRF53_SPI4_MASTER
      if (g_spi4dev.initialized)
        {
          nrf53_spi_init(&g_spi4dev);
        }
#endif
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_spibus_initialize
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

struct spi_dev_s *nrf53_spibus_initialize(int port)
{
  struct nrf53_spidev_s *priv = NULL;

  /* Get SPI driver data */

  switch (port)
    {
#ifdef CONFIG_NRF53_SPI0_MASTER
      case 0:
        {
          priv = &g_spi0dev;
          break;
        }
#endif

#ifdef CONFIG_NRF53_SPI1_MASTER
      case 1:
        {
          priv = &g_spi1dev;
          break;
        }
#endif

#ifdef CONFIG_NRF53_SPI2_MASTER
      case 2:
        {
          priv = &g_spi2dev;
          break;
        }
#endif

#ifdef CONFIG_NRF53_SPI3_MASTER
      case 3:
        {
          priv = &g_spi3dev;
          break;
        }
#endif

#ifdef CONFIG_NRF53_SPI4_MASTER
      case 4:
        {
          priv = &g_spi4dev;
          break;
        }
#endif

      default:
        {
          goto errout;
        }
    }

  /* Initialize the SPI */

  nrf53_spi_init(priv);

  /* Mark device as initialized */

  priv->initialized = true;

#ifdef CONFIG_NRF53_SPI_MASTER_INTERRUPTS
  /* Attach SPI interrupt */

  irq_attach(priv->irq, nrf53_spi_isr, priv);
  up_enable_irq(priv->irq);
#endif

errout:
  return (struct spi_dev_s *)priv;
}
