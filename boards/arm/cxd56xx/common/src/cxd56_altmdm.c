/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_altmdm.c
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

#if defined(CONFIG_MODEM_ALTMDM) && defined(CONFIG_CXD56_LTE) && \
  defined(CONFIG_CXD56_GPIO_IRQ) && defined(CONFIG_CXD56_SPI)

#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/arch.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/modem/altmdm.h>

#include <arch/board/board.h>
#include <arch/board/cxd56_altmdm.h>

#include "cxd56_spi.h"
#include "cxd56_dmac.h"
#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE)
#  error CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE is not set
#endif

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

#define WAIT_READY_TO_GPIO_INTERRUPT 300 /* micro seconds */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void altmdm_poweron(void);
static void altmdm_poweroff(void);
static void altmdm_sready_irqattach(bool attach, xcpt_t handler);
static void altmdm_sready_irqenable(bool enable);
static bool altmdm_sready(void);
static void altmdm_master_request(bool request);
static void altmdm_wakeup(bool wakeup);
static uint32_t altmdm_spi_maxfreq(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void *g_devhandle = NULL;
static const struct altmdm_lower_s g_altmdm_lower =
{
  .poweron          = altmdm_poweron,
  .poweroff         = altmdm_poweroff,
  .sready_irqattach = altmdm_sready_irqattach,
  .sready_irqenable = altmdm_sready_irqenable,
  .sready           = altmdm_sready,
  .master_request   = altmdm_master_request,
  .wakeup           = altmdm_wakeup,
  .spi_maxfreq      = altmdm_spi_maxfreq
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
 * Name: altmdm_poweron
 *
 * Description:
 *   Power on the Altair modem device on the board.
 *
 ****************************************************************************/

static void altmdm_poweron(void)
{
  /* power on altair modem device */

  board_altmdm_poweron();

  /* Input enable */

  cxd56_gpio_config(ALTMDM_SLAVE_REQ, true);

  /* Output enable */

  cxd56_gpio_config(ALTMDM_MASTER_REQ, false);
  cxd56_gpio_config(ALTMDM_WAKEUP, false);

  /* Write a default value for output pin */

  cxd56_gpio_write(ALTMDM_MASTER_REQ, false);
  cxd56_gpio_write(ALTMDM_WAKEUP, false);

  /* Slave request seems to float in Lite Hibernation and becomes HIGH at
   * some times when it should stay LOW.
   */

  cxd56_pin_config(PINCONF_SET(ALTMDM_SLAVE_REQ,
                               PINCONF_MODE0,
                               PINCONF_INPUT_ENABLE,
                               PINCONF_DRIVE_NORMAL, PINCONF_PULLDOWN));

  /* enable the SPI pin */

  spi_pincontrol(SPI_CH, true);
}

/****************************************************************************
 * Name: altmdm_poweroff
 *
 * Description:
 *   Power off the Altair modem device on the board.
 *
 ****************************************************************************/

static void altmdm_poweroff(void)
{
  /* disable the SPI pin */

  spi_pincontrol(SPI_CH, false);

  /* Input disable */

  cxd56_gpio_config(ALTMDM_SLAVE_REQ, false);

  /* Output disable(Hi-z) */

  cxd56_gpio_config(ALTMDM_MASTER_REQ, false);
  cxd56_gpio_config(ALTMDM_WAKEUP, false);

  /* power off Altair modem device */

  board_altmdm_poweroff();
}

/****************************************************************************
 * Name: altmdm_sready_irqattach
 *
 * Description:
 *   Register Slave-Request GPIO irq.
 *
 ****************************************************************************/

static void altmdm_sready_irqattach(bool attach, xcpt_t handler)
{
  uint32_t pol = GPIOINT_LEVEL_HIGH;
  uint32_t nf = GPIOINT_NOISE_FILTER_DISABLE;

  if (attach)
    {
      /* Attach then enable the new interrupt handler */

      cxd56_gpioint_config(ALTMDM_SLAVE_REQ,
                           (GPIOINT_TOGGLE_MODE_MASK | nf | pol),
                           handler, NULL);
    }
  else
    {
      /* Disable the interrupt handler */

      cxd56_gpioint_config(ALTMDM_SLAVE_REQ, 0, NULL, NULL);
    }
}

/****************************************************************************
 * Name: altmdm_sready_irqenable
 *
 * Description:
 *   Enable or disable Slave-Request GPIO interrupt.
 *
 ****************************************************************************/

static void altmdm_sready_irqenable(bool enable)
{
  if (enable)
    {
      /* enable interrupt */

      cxd56_gpioint_enable(ALTMDM_SLAVE_REQ);
    }
  else
    {
      /* disable interrupt */

      cxd56_gpioint_disable(ALTMDM_SLAVE_REQ);
    }
}

/****************************************************************************
 * Name: altmdm_sready
 *
 * Description:
 *   Read Slave-Request GPIO pin.
 *
 ****************************************************************************/

static bool altmdm_sready(void)
{
  return cxd56_gpio_read(ALTMDM_SLAVE_REQ);
}

/****************************************************************************
 * Name: altmdm_master_request
 *
 * Description:
 *   Write Master-Request GPIO pin.
 *
 ****************************************************************************/

static void altmdm_master_request(bool request)
{
  /* If the GPIO falls within 300us after raising
   * (or GPIO raises within 300us after falling), the modem may miss the GPIO
   * interrupt. So delay by 300us before changing the GPIO.
   */

  up_udelay(WAIT_READY_TO_GPIO_INTERRUPT);
  cxd56_gpio_write(ALTMDM_MASTER_REQ, request);
}

/****************************************************************************
 * Name: altmdm_wakeup
 *
 * Description:
 *   Write Modme-Wakeup GPIO pin.
 *
 ****************************************************************************/

static void altmdm_wakeup(bool wakeup)
{
  cxd56_gpio_write(ALTMDM_WAKEUP, wakeup);
}

/****************************************************************************
 * Name: altmdm_spi_maxfreq
 *
 * Description:
 *   Get the maximum SPI clock frequency.
 *
 ****************************************************************************/

static uint32_t altmdm_spi_maxfreq(void)
{
  return SPI_MAXFREQUENCY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_altmdm_initialize
 *
 * Description:
 *   Initialize Altair modem
 *
 ****************************************************************************/

int board_altmdm_initialize(const char *devpath)
{
  struct spi_dev_s *spi;
#if defined(CONFIG_CXD56_LTE_SPI4_DMAC) || defined(CONFIG_CXD56_LTE_SPI5_DMAC)
  DMA_HANDLE            hdl;
  dma_config_t          conf;
#endif

  m_info("Initializing ALTMDM..\n");

  if (!g_devhandle)
    {
      /* Initialize spi deivce */

      spi = cxd56_spibus_initialize(SPI_CH);
      if (!spi)
        {
          m_err("ERROR: Failed to initialize spi%d.\n", SPI_CH);
          return -ENODEV;
        }

#if defined(CONFIG_CXD56_LTE_SPI4_DMAC) || defined(CONFIG_CXD56_LTE_SPI5_DMAC)
      hdl = cxd56_dmachannel(DMA_TXCH,
                             CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE);
      if (hdl)
        {
          conf.channel_cfg = DMA_TXCHCHG;
          conf.dest_width  = CXD56_DMAC_WIDTH8;
          conf.src_width   = CXD56_DMAC_WIDTH8;
          cxd56_spi_dmaconfig(SPI_CH, CXD56_SPI_DMAC_CHTYPE_TX, hdl,
                              &conf);
        }

      hdl = cxd56_dmachannel(DMA_RXCH,
                             CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE);
      if (hdl)
        {
          conf.channel_cfg = DMA_RXCHCFG;
          conf.dest_width  = CXD56_DMAC_WIDTH8;
          conf.src_width   = CXD56_DMAC_WIDTH8;
          cxd56_spi_dmaconfig(SPI_CH, CXD56_SPI_DMAC_CHTYPE_RX, hdl,
                              &conf);
        }
#endif

      spi_pincontrol(SPI_CH, false);

      g_devhandle = altmdm_register(devpath, spi, &g_altmdm_lower);
      if (!g_devhandle)
        {
          m_err("ERROR: Failed to register altmdm driver.\n");
          return -ENODEV;
        }

      board_altmdm_poweroff();
    }

  return OK;
}

/****************************************************************************
 * Name: board_altmdm_uninitialize
 *
 * Description:
 *   Uninitialize Altair modem
 *
 ****************************************************************************/

int board_altmdm_uninitialize(void)
{
  m_info("Uninitializing ALTMDM..\n");

  if (g_devhandle)
    {
      altmdm_unregister(g_devhandle);

      g_devhandle = NULL;
    }

  return OK;
}

#endif
