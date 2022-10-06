/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_alt1250.c
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

#if defined(CONFIG_MODEM_ALT1250) && defined(CONFIG_CXD56_LTE) && \
  defined(CONFIG_CXD56_GPIO_IRQ) && defined(CONFIG_CXD56_SPI)

#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/arch.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/modem/alt1250.h>

#include <arch/board/board.h>
#include <arch/board/cxd56_alt1250.h>

#include "cxd56_spi.h"
#include "cxd56_dmac.h"
#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_PACKET_SIZE 2064

#if defined(CONFIG_CXD56_LTE_SPI4)
#  define SPI_CH           (4)
#  define SPI_MAXFREQUENCY (9750000)
#  if  defined(CONFIG_CXD56_LTE_SPI4_DMAC)
#    define DMA_TXCH       (2)
#    define DMA_RXCH       (3)
#    define DMA_TXCHCHG    (CXD56_DMA_PERIPHERAL_SPI4_TX)
#    define DMA_RXCHCFG    (CXD56_DMA_PERIPHERAL_SPI4_RX)
#  endif
#elif defined(CONFIG_CXD56_LTE_SPI5)
#  define SPI_CH           (5)
#  define SPI_MAXFREQUENCY (13000000)
#  if  defined(CONFIG_CXD56_LTE_SPI5_DMAC)
#    define DMA_TXCH       (4)
#    define DMA_RXCH       (5)
#    define DMA_TXCHCHG    (CXD56_DMA_PERIPHERAL_SPI5_TX)
#    define DMA_RXCHCFG    (CXD56_DMA_PERIPHERAL_SPI5_RX)
#  endif
#else
#  error "Select LTE SPI 4 or 5"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static struct spi_dev_s *alt1250_poweron(void);
static void alt1250_poweroff(void);
static void alt1250_reset(void);
static void alt1250_irqattach(xcpt_t handler);
static void alt1250_irqenable(bool enable);
static bool alt1250_get_sready(void);
static void alt1250_set_mready(bool on);
static void alt1250_set_wakeup(bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void *g_devhandle = NULL;
static const struct alt1250_lower_s g_alt1250_lower =
{
  .poweron      = alt1250_poweron,
  .poweroff     = alt1250_poweroff,
  .reset        = alt1250_reset,
  .irqattach    = alt1250_irqattach,
  .irqenable    = alt1250_irqenable,
  .get_sready   = alt1250_get_sready,
  .set_mready   = alt1250_set_mready,
  .set_wakeup   = alt1250_set_wakeup,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_pincontrol
 *
 * Description:
 *   Configure the SPI pin
 *
 * Input Parameter:
 *   bus - SPI bus number to control
 *   on - true: enable pin, false: disable pin
 *
 ****************************************************************************/

static void spi_pincontrol(int bus, bool on)
{
  switch (bus)
    {
#ifdef CONFIG_CXD56_SPI4
      case 4:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI4);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI4_GPIO);
          }
        break;
#endif /* CONFIG_CXD56_SPI4 */

#ifdef CONFIG_CXD56_SPI5
      case 5:
#ifdef CONFIG_CXD56_SPI5_PINMAP_EMMC
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_EMMCA_SPI5);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_EMMCA_GPIO);
          }
#endif /* CONFIG_CXD56_SPI5_PINMAP_EMMC */

#ifdef CONFIG_CXD56_SPI5_PINMAP_SDIO
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SDIOA_SPI5);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SDIOA_GPIO);
          }
#endif /* CONFIG_CXD56_SPI5_PINMAP_SDIO */
        break;
#endif /* CONFIG_CXD56_SPI5 */
      default:
        break;
    }
}

/****************************************************************************
 * Name: set_spiparam
 *
 * Description:
 *   Setup the SPI parameters.
 *
 ****************************************************************************/

static void set_spiparam(struct spi_dev_s *spidev)
{
  SPI_LOCK(spidev, true);
  SPI_SETMODE(spidev, SPIDEV_MODE0);
  SPI_SETBITS(spidev, 8);
  SPI_SETFREQUENCY(spidev, SPI_MAXFREQUENCY);
  SPI_LOCK(spidev, false);
}

/****************************************************************************
 * Name: alt1250_poweron
 *
 * Description:
 *   Power on the Altair modem device on the board.
 *
 ****************************************************************************/

static struct spi_dev_s *alt1250_poweron(void)
{
  struct spi_dev_s *spi;
#if defined(CONFIG_CXD56_LTE_SPI4_DMAC) || defined(CONFIG_CXD56_LTE_SPI5_DMAC)
  DMA_HANDLE            hdl;
  dma_config_t          conf;
#endif

  /* Initialize spi deivce */

  spi = cxd56_spibus_initialize(SPI_CH);
  if (!spi)
    {
      m_err("ERROR: Failed to initialize spi%d.\n", SPI_CH);
      return NULL;
    }

#if defined(CONFIG_CXD56_LTE_SPI4_DMAC) || defined(CONFIG_CXD56_LTE_SPI5_DMAC)
  hdl = cxd56_dmachannel(DMA_TXCH, MAX_PACKET_SIZE);
  if (hdl)
    {
      conf.channel_cfg = DMA_TXCHCHG;
      conf.dest_width  = CXD56_DMAC_WIDTH8;
      conf.src_width   = CXD56_DMAC_WIDTH8;
      cxd56_spi_dmaconfig(SPI_CH, CXD56_SPI_DMAC_CHTYPE_TX, hdl, &conf);
    }

  hdl = cxd56_dmachannel(DMA_RXCH, MAX_PACKET_SIZE);
  if (hdl)
    {
      conf.channel_cfg = DMA_RXCHCFG;
      conf.dest_width  = CXD56_DMAC_WIDTH8;
      conf.src_width   = CXD56_DMAC_WIDTH8;
      cxd56_spi_dmaconfig(SPI_CH, CXD56_SPI_DMAC_CHTYPE_RX, hdl, &conf);
    }
#endif

  /* power on altair modem device */

  board_alt1250_poweron();

  /* Input enable */

  cxd56_gpio_config(ALT1250_SLAVE_REQ, true);

  /* Output enable */

  cxd56_gpio_config(ALT1250_MASTER_REQ, false);
  cxd56_gpio_config(ALT1250_WAKEUP, false);

  /* Write a default value for output pin */

  cxd56_gpio_write(ALT1250_MASTER_REQ, false);
  cxd56_gpio_write(ALT1250_WAKEUP, false);

  /* Slave request seems to float in Lite Hibernation and becomes HIGH at
   * some times when it should stay LOW.
   */

  cxd56_pin_config(PINCONF_SET(ALT1250_SLAVE_REQ,
                               PINCONF_MODE0,
                               PINCONF_INPUT_ENABLE,
                               PINCONF_DRIVE_NORMAL, PINCONF_PULLDOWN));

  /* enable the SPI pin */

  spi_pincontrol(SPI_CH, true);

  set_spiparam(spi);

  return spi;
}

/****************************************************************************
 * Name: alt1250_poweroff
 *
 * Description:
 *   Power off the Altair modem device on the board.
 *
 ****************************************************************************/

static void alt1250_poweroff(void)
{
  /* disable the SPI pin */

  spi_pincontrol(SPI_CH, false);

  /* Input disable */

  cxd56_gpio_config(ALT1250_SLAVE_REQ, false);

  /* Output disable(Hi-z) */

  cxd56_gpio_config(ALT1250_MASTER_REQ, false);
  cxd56_gpio_config(ALT1250_WAKEUP, false);

  /* power off Altair modem device */

  board_alt1250_poweroff();
}

/****************************************************************************
 * Name: alt1250_reset
 *
 * Description:
 *   Reset the Altair modem device on the board.
 *
 ****************************************************************************/

static void alt1250_reset(void)
{
  /* Reset Altair modem device */

  board_alt1250_reset();
}

/****************************************************************************
 * Name: alt1250_irqattach
 *
 * Description:
 *   Register Slave-Request GPIO irq.
 *
 ****************************************************************************/

static void alt1250_irqattach(xcpt_t handler)
{
  uint32_t pol = GPIOINT_LEVEL_HIGH;
  uint32_t nf = GPIOINT_NOISE_FILTER_DISABLE;

  if (handler)
    {
      /* Attach then enable the new interrupt handler */

      cxd56_gpioint_config(ALT1250_SLAVE_REQ,
                           (GPIOINT_TOGGLE_MODE_MASK | nf | pol),
                           handler, NULL);
    }
  else
    {
      /* Disable the interrupt handler */

      cxd56_gpioint_config(ALT1250_SLAVE_REQ, 0, NULL, NULL);
    }
}

/****************************************************************************
 * Name: alt1250_irqenable
 *
 * Description:
 *   Enable or disable Slave-Request GPIO interrupt.
 *
 ****************************************************************************/

static void alt1250_irqenable(bool enable)
{
  if (enable)
    {
      /* enable interrupt */

      cxd56_gpioint_enable(ALT1250_SLAVE_REQ);
    }
  else
    {
      /* disable interrupt */

      cxd56_gpioint_disable(ALT1250_SLAVE_REQ);
    }
}

/****************************************************************************
 * Name: alt1250_get_sready
 *
 * Description:
 *   Read Slave-Request GPIO pin.
 *
 ****************************************************************************/

static bool alt1250_get_sready(void)
{
  return cxd56_gpio_read(ALT1250_SLAVE_REQ);
}

/****************************************************************************
 * Name: alt1250_master_request
 *
 * Description:
 *   Write Master-Request GPIO pin.
 *
 ****************************************************************************/

static void alt1250_set_mready(bool on)
{
  cxd56_gpio_write(ALT1250_MASTER_REQ, on);
}

/****************************************************************************
 * Name: alt1250_wakeup
 *
 * Description:
 *   Write Modme-Wakeup GPIO pin.
 *
 ****************************************************************************/

static void alt1250_set_wakeup(bool on)
{
  cxd56_gpio_write(ALT1250_WAKEUP, on);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_alt1250_initialize
 *
 * Description:
 *   Initialize Altair modem
 *
 ****************************************************************************/

int board_alt1250_initialize(const char *devpath)
{
  m_info("Initializing ALT1250..\n");

  if (!g_devhandle)
    {
      g_devhandle = alt1250_register(devpath, NULL, &g_alt1250_lower);
      if (!g_devhandle)
        {
          m_err("ERROR: Failed to register alt1250 driver.\n");
          return -ENODEV;
        }
    }

  return OK;
}

#endif
