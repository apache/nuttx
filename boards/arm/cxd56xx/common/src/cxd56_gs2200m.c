/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_gs2200m.c
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

#include <debug.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/gs2200m.h>

#include <arch/chip/pin.h>

#include "cxd56_pinconfig.h"
#include "cxd56_spi.h"
#include "cxd56_dmac.h"
#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"

#define DMA_TXCH       (CONFIG_CXD56_DMAC_SPI5_TX_CH)
#define DMA_RXCH       (CONFIG_CXD56_DMAC_SPI5_RX_CH)
#define DMA_TXCH_CFG   (CXD56_DMA_PERIPHERAL_SPI5_TX)
#define DMA_RXCH_CFG   (CXD56_DMA_PERIPHERAL_SPI5_RX)
#define SPI_TX_MAXSIZE (CONFIG_CXD56_DMAC_SPI5_TX_MAXSIZE)
#define SPI_RX_MAXSIZE (CONFIG_CXD56_DMAC_SPI5_RX_MAXSIZE)

#if defined(CONFIG_WIFI_BOARD_IS110B_HARDWARE_VERSION_10B)
/* v1.0b */
#define GS2200M_GPIO_37          (PIN_UART2_CTS)
#define GS2200M_EXT_RTC_RESET_IN (PIN_EMMC_DATA3)
#elif defined(CONFIG_WIFI_BOARD_IS110B_HARDWARE_VERSION_10C)
/* v1.0c */
#define GS2200M_GPIO_37          (PIN_EMMC_DATA2)
#define GS2200M_EXT_RTC_RESET_IN (PIN_EMMC_DATA3)
#else
/* v1.0a */
#define GS2200M_GPIO_37          (PIN_UART2_CTS)
#define GS2200M_EXT_RTC_RESET_IN (PIN_UART2_RTS)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  gs2200m_irq_attach(xcpt_t, void *);
static void gs2200m_irq_enable(void);
static void gs2200m_irq_disable(void);
static uint32_t gs2200m_dready(int *);
static void gs2200m_reset(bool);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gs2200m_lower_s g_wifi_lower =
{
  .attach  = gs2200m_irq_attach,
  .enable  = gs2200m_irq_enable,
  .disable = gs2200m_irq_disable,
  .dready  = gs2200m_dready,
  .reset   = gs2200m_reset
};

static void *g_devhandle = NULL;
static volatile int32_t  _enable_count = 0;
static volatile uint32_t _n_called;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gs2200m_irq_attach
 ****************************************************************************/

static int gs2200m_irq_attach(xcpt_t handler, void *arg)
{
  cxd56_gpioint_config(GS2200M_GPIO_37,
                       GPIOINT_LEVEL_HIGH,
                       handler,
                       arg);
  return 0;
}

/****************************************************************************
 * Name: gs2200m_irq_enable
 ****************************************************************************/

static void gs2200m_irq_enable(void)
{
  irqstate_t flags = enter_critical_section();

  wlinfo("== ec:%" PRId32 " called=%" PRId32 "\n",
         _enable_count, _n_called++);

  if (0 == _enable_count)
    {
      cxd56_gpioint_enable(GS2200M_GPIO_37);
    }

  _enable_count++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: gs2200m_irq_disable
 ****************************************************************************/

static void gs2200m_irq_disable(void)
{
  irqstate_t flags = enter_critical_section();

  wlinfo("== ec:%" PRId32 " called=%" PRId32 "\n",
         _enable_count, _n_called++);

  _enable_count--;

  if (0 == _enable_count)
    {
      cxd56_gpioint_disable(GS2200M_GPIO_37);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: gs2200m_dready
 ****************************************************************************/

static uint32_t gs2200m_dready(int *ec)
{
  irqstate_t flags = enter_critical_section();

  uint32_t r = cxd56_gpio_read(GS2200M_GPIO_37);

  if (ec)
    {
      /* Copy enable count (just for debug) */

      *ec = _enable_count;
    }

  leave_critical_section(flags);
  return r;
}

/****************************************************************************
 * Name: gs2200m_reset
 ****************************************************************************/

static void gs2200m_reset(bool reset)
{
  cxd56_gpio_write(GS2200M_EXT_RTC_RESET_IN, !reset);
}

/****************************************************************************
 * Name: spi_pincontrol
 *
 * Description:
 *   Configure the SPI pin
 *
 * Input Parameter:
 *   on - true: enable pin, false: disable pin
 ****************************************************************************/

static void spi_pincontrol(int bus, bool on)
{
  if (bus == 5)
    {
#ifdef CONFIG_CXD56_SPI5_PINMAP_EMMC
      if (on)
        {
          CXD56_PIN_CONFIGS(PINCONFS_EMMCA_SPI5);
        }
      else
        {
          CXD56_PIN_CONFIGS(PINCONFS_EMMCA_GPIO);
        }
#endif

#ifdef CONFIG_CXD56_SPI5_PINMAP_SDIO
      if (on)
        {
          CXD56_PIN_CONFIGS(PINCONFS_SDIOA_SPI5);
        }
      else
        {
          CXD56_PIN_CONFIGS(PINCONFS_SDIOA_GPIO);
        }
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gs2200m_initialize
 ****************************************************************************/

int board_gs2200m_initialize(const char *devpath, int bus)
{
  struct spi_dev_s *spi;
  DMA_HANDLE    hdl;
  dma_config_t  conf;

  wlinfo("Initializing GS2200M..\n");

  if (!g_devhandle)
    {
      /* Change UART2 to GPIO */

      CXD56_PIN_CONFIGS(PINCONFS_UART2_GPIO);
      cxd56_gpio_config(GS2200M_GPIO_37, true);
      cxd56_gpio_config(GS2200M_EXT_RTC_RESET_IN, false);

      /* Initialize spi device */

      spi = cxd56_spibus_initialize(bus);

      if (!spi)
        {
          wlerr("ERROR: Failed to initialize spi%d.\n", bus);
          return -ENODEV;
        }

      hdl = cxd56_dmachannel(DMA_TXCH, SPI_TX_MAXSIZE);
      if (hdl)
        {
          conf.channel_cfg = DMA_TXCH_CFG;
          conf.dest_width  = CXD56_DMAC_WIDTH8;
          conf.src_width   = CXD56_DMAC_WIDTH8;
          cxd56_spi_dmaconfig(bus, CXD56_SPI_DMAC_CHTYPE_TX, hdl, &conf);
        }

      hdl = cxd56_dmachannel(DMA_RXCH, SPI_RX_MAXSIZE);
      if (hdl)
        {
          conf.channel_cfg = DMA_RXCH_CFG;
          conf.dest_width  = CXD56_DMAC_WIDTH8;
          conf.src_width   = CXD56_DMAC_WIDTH8;
          cxd56_spi_dmaconfig(bus, CXD56_SPI_DMAC_CHTYPE_RX, hdl, &conf);
        }

      /* Enable SPI5 */

      spi_pincontrol(bus, true);

      g_devhandle = gs2200m_register(devpath, spi, &g_wifi_lower);

      if (!g_devhandle)
        {
          wlerr("ERROR: Failed to register gs2200m driver.\n");
          return -ENODEV;
        }
    }

  return OK;
}
