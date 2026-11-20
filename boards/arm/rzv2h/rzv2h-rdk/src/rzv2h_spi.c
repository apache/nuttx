/****************************************************************************
 * boards/arm/rzv2h/rzv2h-rdk/src/rzv2h_spi.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#if defined(CONFIG_RZV2H_SPI_CHANNEL_0_MASTER) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_1_MASTER) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_2_MASTER)
#  define HAVE_RZV2H_SPI_MASTER 1
#endif

#if defined(CONFIG_SPI_SLAVE) && \
   (defined(CONFIG_RZV2H_SPI_CHANNEL_0_SLAVE) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_1_SLAVE) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_2_SLAVE))
#  define HAVE_RZV2H_SPI_SLAVE 1
#endif

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/spi/spi.h>
#ifdef HAVE_RZV2H_SPI_SLAVE
#  include <nuttx/spi/slave.h>
#endif
#ifdef CONFIG_SPI_DRIVER
#  include <nuttx/spi/spi_transfer.h>
#endif

#include <arch/board/board.h>

#include "rzv2h_gpio.h"
#include "rzv2h_spi.h"
#include "rzv2h-rdk.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RZV2H_SPI_CHANNEL_0_MASTER
static struct spi_dev_s *g_spi0;
#elif defined(HAVE_RZV2H_SPI_SLAVE) && \
      defined(CONFIG_RZV2H_SPI_CHANNEL_0_SLAVE)
static struct spi_slave_ctrlr_s *g_spi_slave0;
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_1_MASTER
static struct spi_dev_s *g_spi1;
#elif defined(HAVE_RZV2H_SPI_SLAVE) && \
      defined(CONFIG_RZV2H_SPI_CHANNEL_1_SLAVE)
static struct spi_slave_ctrlr_s *g_spi_slave1;
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_2_MASTER
static struct spi_dev_s *g_spi2;
#elif defined(HAVE_RZV2H_SPI_SLAVE) && \
      defined(CONFIG_RZV2H_SPI_CHANNEL_2_SLAVE)
static struct spi_slave_ctrlr_s *g_spi_slave2;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_RZV2H_SPI_CHANNEL_0) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_1) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_2)
/****************************************************************************
 * Name: rzv2h_spi_pin_initialize
 *
 * Description:
 *   Configure the selected SSL, MISO, MOSI, and SCK pins for one SPI_B
 *   channel.  SSL is skipped when the configured clock-synchronous mode
 *   does not use it.
 *
 * Input Parameters:
 *   channel  - SPI_B channel number used in diagnostic messages.
 *   ssl      - Selected SSL pin configuration.
 *   ssl_used - True when the channel uses the SSL signal.
 *   miso     - MISO pin configuration.
 *   mosi     - MOSI pin configuration.
 *   sck      - SCK pin configuration.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on pin-configuration failure.
 *
 ****************************************************************************/

static int rzv2h_spi_pin_initialize(int channel, gpio_pinset_t ssl,
                                    bool ssl_used,
                                    gpio_pinset_t miso, gpio_pinset_t mosi,
                                    gpio_pinset_t sck)
{
  int ret;

  /* Configure SSL as a native peripheral output for a 4-wire master or as
   * a native peripheral input for a 4-wire slave.  3-wire mode does not use
   * SSL.
   */

  if (ssl_used)
    {
      ret = rzv2h_configgpio(ssl);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to configure SPI%d SSL: %d\n",
                 channel, ret);
          return ret;
        }
    }

  ret = rzv2h_configgpio(miso);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure SPI%d MISO: %d\n",
             channel, ret);
      return ret;
    }

  ret = rzv2h_configgpio(mosi);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure SPI%d MOSI: %d\n",
             channel, ret);
      return ret;
    }

  ret = rzv2h_configgpio(sck);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to configure SPI%d SCK: %d\n",
             channel, ret);
    }

  return ret;
}

#ifdef HAVE_RZV2H_SPI_MASTER
/****************************************************************************
 * Name: rzv2h_spi_master_channel
 *
 * Description:
 *   Identify the enabled master channel represented by a lower-half device.
 *
 * Input Parameters:
 *   dev - SPI master lower-half instance.
 *
 * Output Parameters:
 *   ssl_used - Receives true when the identified channel uses SSL.
 *
 * Returned Value:
 *   The SPI_B channel number on success; -ENODEV if dev is not an enabled
 *   board SPI master instance.
 *
 ****************************************************************************/

static int rzv2h_spi_master_channel(struct spi_dev_s *dev,
                                    bool *ssl_used)
{
#ifdef CONFIG_RZV2H_SPI_CHANNEL_0_MASTER
  if (dev == g_spi0)
    {
      *ssl_used = BOARD_SPI0_SSL_USED != 0;
      return 0;
    }
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_1_MASTER
  if (dev == g_spi1)
    {
      *ssl_used = BOARD_SPI1_SSL_USED != 0;
      return 1;
    }
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_2_MASTER
  if (dev == g_spi2)
    {
      *ssl_used = BOARD_SPI2_SSL_USED != 0;
      return 2;
    }
#endif

  return -ENODEV;
}

/****************************************************************************
 * Name: rzv2h_spi_master_bus_initialize
 *
 * Description:
 *   Configure one board SPI_B master channel, initialize its lower half,
 *   and optionally register its character device.
 *
 * Input Parameters:
 *   channel  - SPI_B channel number.
 *   ssl      - Selected SSL pin configuration.
 *   ssl_used - True when the channel uses the SSL signal.
 *   miso     - MISO pin configuration.
 *   mosi     - MOSI pin configuration.
 *   sck      - SCK pin configuration.
 *
 * Output Parameters:
 *   bus - Receives the initialized SPI master lower-half instance.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rzv2h_spi_master_bus_initialize(int channel, gpio_pinset_t ssl,
                                           bool ssl_used,
                                           gpio_pinset_t miso,
                                           gpio_pinset_t mosi,
                                           gpio_pinset_t sck,
                                           struct spi_dev_s **bus)
{
  struct spi_dev_s *spi;
  int ret;

  if (*bus != NULL)
    {
      return OK;
    }

  ret = rzv2h_spi_pin_initialize(channel, ssl, ssl_used, miso, mosi, sck);
  if (ret < 0)
    {
      return ret;
    }

  spi = rzv2h_spibus_initialize(channel);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI-B channel %d\n",
             channel);
      return -ENODEV;
    }

#ifdef CONFIG_SPI_DRIVER
  ret = spi_register(spi, channel);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/spi%d: %d\n",
             channel, ret);
      return ret;
    }
#endif

  *bus = spi;
  return OK;
}
#endif

#ifdef HAVE_RZV2H_SPI_SLAVE
/****************************************************************************
 * Name: rzv2h_spi_slave_bus_initialize
 *
 * Description:
 *   Configure one board SPI_B slave channel, initialize its controller,
 *   and optionally register its character device.
 *
 * Input Parameters:
 *   channel  - SPI_B channel number.
 *   ssl      - Selected SSL pin configuration.
 *   ssl_used - True when the channel uses the SSL signal.
 *   miso     - MISO pin configuration.
 *   mosi     - MOSI pin configuration.
 *   sck      - SCK pin configuration.
 *
 * Output Parameters:
 *   ctrlr_out - Receives the initialized SPI slave controller instance.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rzv2h_spi_slave_bus_initialize(
  int channel, gpio_pinset_t ssl, bool ssl_used, gpio_pinset_t miso,
  gpio_pinset_t mosi, gpio_pinset_t sck,
  struct spi_slave_ctrlr_s **ctrlr_out)
{
  struct spi_slave_ctrlr_s *ctrlr;
  int ret;

  if (*ctrlr_out != NULL)
    {
      return OK;
    }

  ret = rzv2h_spi_pin_initialize(channel, ssl, ssl_used, miso, mosi, sck);
  if (ret < 0)
    {
      return ret;
    }

  ctrlr = rzv2h_spi_slave_initialize(channel);
  if (ctrlr == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize SPI-B slave channel %d\n",
             channel);
      return -ENODEV;
    }

#ifdef CONFIG_SPI_SLAVE_DRIVER
  ret = spi_slave_register(ctrlr, channel);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/spislv%d: %d\n",
             channel, ret);
      return ret;
    }
#endif

  *ctrlr_out = ctrlr;
  return OK;
}
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RZV2H_SPI_CHANNEL_0_MASTER
/****************************************************************************
 * Name: rzv2h_spi0status
 *
 * Description:
 *   Report the board-defined status of a device on SPI_B channel 0.
 *
 * Input Parameters:
 *   dev   - SPI lower-half device instance; unused.
 *   devid - NuttX SPI device identifier.
 *
 * Returned Value:
 *   SPI_STATUS_PRESENT for SPIDEV_USER(0); zero for any other device.
 *
 ****************************************************************************/

uint8_t rzv2h_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  UNUSED(dev);
  return devid == SPIDEV_USER(0) ? SPI_STATUS_PRESENT : 0;
}

#ifdef CONFIG_SPI_CMDDATA
/****************************************************************************
 * Name: rzv2h_spi0cmddata
 *
 * Description:
 *   Report that channel 0 has no board command/data control signal.
 *
 * Input Parameters:
 *   dev   - SPI lower-half device instance; unused.
 *   devid - NuttX SPI device identifier.
 *   cmd   - Requested command state; unused.
 *
 * Returned Value:
 *   -ENOSYS for SPIDEV_USER(0); -ENODEV for any other device.
 *
 ****************************************************************************/

int rzv2h_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  UNUSED(dev);
  UNUSED(cmd);
  return devid == SPIDEV_USER(0) ? -ENOSYS : -ENODEV;
}
#endif
#endif /* CONFIG_RZV2H_SPI_CHANNEL_0_MASTER */

#ifdef CONFIG_RZV2H_SPI_CHANNEL_1_MASTER
/****************************************************************************
 * Name: rzv2h_spi1status
 *
 * Description:
 *   Report the board-defined status of a device on SPI_B channel 1.
 *
 * Input Parameters:
 *   dev   - SPI lower-half device instance; unused.
 *   devid - NuttX SPI device identifier.
 *
 * Returned Value:
 *   SPI_STATUS_PRESENT for SPIDEV_USER(0); zero for any other device.
 *
 ****************************************************************************/

uint8_t rzv2h_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  UNUSED(dev);
  return devid == SPIDEV_USER(0) ? SPI_STATUS_PRESENT : 0;
}

#ifdef CONFIG_SPI_CMDDATA
/****************************************************************************
 * Name: rzv2h_spi1cmddata
 *
 * Description:
 *   Report that channel 1 has no board command/data control signal.
 *
 * Input Parameters:
 *   dev   - SPI lower-half device instance; unused.
 *   devid - NuttX SPI device identifier.
 *   cmd   - Requested command state; unused.
 *
 * Returned Value:
 *   -ENOSYS for SPIDEV_USER(0); -ENODEV for any other device.
 *
 ****************************************************************************/

int rzv2h_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  UNUSED(dev);
  UNUSED(cmd);
  return devid == SPIDEV_USER(0) ? -ENOSYS : -ENODEV;
}
#endif
#endif /* CONFIG_RZV2H_SPI_CHANNEL_1_MASTER */

#ifdef CONFIG_RZV2H_SPI_CHANNEL_2_MASTER
/****************************************************************************
 * Name: rzv2h_spi2status
 *
 * Description:
 *   Report the board-defined status of a device on SPI_B channel 2.
 *
 * Input Parameters:
 *   dev   - SPI lower-half device instance; unused.
 *   devid - NuttX SPI device identifier.
 *
 * Returned Value:
 *   SPI_STATUS_PRESENT for SPIDEV_USER(0); zero for any other device.
 *
 ****************************************************************************/

uint8_t rzv2h_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  UNUSED(dev);
  return devid == SPIDEV_USER(0) ? SPI_STATUS_PRESENT : 0;
}

#ifdef CONFIG_SPI_CMDDATA
/****************************************************************************
 * Name: rzv2h_spi2cmddata
 *
 * Description:
 *   Report that channel 2 has no board command/data control signal.
 *
 * Input Parameters:
 *   dev   - SPI lower-half device instance; unused.
 *   devid - NuttX SPI device identifier.
 *   cmd   - Requested command state; unused.
 *
 * Returned Value:
 *   -ENOSYS for SPIDEV_USER(0); -ENODEV for any other device.
 *
 ****************************************************************************/

int rzv2h_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  UNUSED(dev);
  UNUSED(cmd);
  return devid == SPIDEV_USER(0) ? -ENOSYS : -ENODEV;
}
#endif
#endif /* CONFIG_RZV2H_SPI_CHANNEL_2_MASTER */

#ifdef HAVE_RZV2H_SPI_MASTER
/****************************************************************************
 * Name: rzv2h_spiselect
 *
 * Description:
 *   Apply the board chip-select policy for any enabled SPI_B master channel.
 *   Clock-synchronous 3-wire mode has no SSL signal.  In 4-wire mode the
 *   selected native SSL pin is driven by SPI_B/FSP during the transfer, so
 *   the NuttX select request requires no GPIO operation.
 *
 * Input Parameters:
 *   dev      - SPI master lower-half instance.
 *   devid    - NuttX SPI device identifier.
 *   selected - True to select the device; false to deselect it.
 *
 * Returned Value:
 *   None.
 ****************************************************************************/

void rzv2h_spiselect(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  bool ssl_used;
  int channel;

  channel = rzv2h_spi_master_channel(dev, &ssl_used);
  if (channel < 0)
    {
      spiwarn("WARNING: Unknown RZ/V2H SPI master device %p\n", dev);
      return;
    }

  if (devid != SPIDEV_USER(0))
    {
      spiwarn("WARNING: SPI%d has no chip select for devid 0x%08lx\n",
              channel, (unsigned long)devid);
      return;
    }

  if (!ssl_used)
    {
      spiinfo("SPI%d ignores select in 3-wire mode\n", channel);
      return;
    }

  spiinfo("SPI%d hardware SSL %s requested\n", channel,
          selected ? "assert" : "deassert");
}
#endif

/****************************************************************************
 * Name: board_spi_initialize
 *
 * Description:
 *   Configure and register all enabled RZ/V2H SPI_B channels.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 ****************************************************************************/

int board_spi_initialize(void)
{
#if defined(CONFIG_RZV2H_SPI_CHANNEL_0) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_1) || \
    defined(CONFIG_RZV2H_SPI_CHANNEL_2)
  int ret;
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_0_MASTER
  ret = rzv2h_spi_master_bus_initialize(0, BOARD_SPI0_SSL_PIN,
                                         BOARD_SPI0_SSL_USED,
                                         BOARD_SPI0_MISO_GPIO,
                                         BOARD_SPI0_MOSI_GPIO,
                                         BOARD_SPI0_SCK_GPIO, &g_spi0);
  if (ret < 0)
    {
      return ret;
    }
#elif defined(HAVE_RZV2H_SPI_SLAVE) && \
      defined(CONFIG_RZV2H_SPI_CHANNEL_0_SLAVE)
  ret = rzv2h_spi_slave_bus_initialize(0, BOARD_SPI0_SSL_PIN,
                                       BOARD_SPI0_SSL_USED,
                                       BOARD_SPI0_MISO_GPIO,
                                       BOARD_SPI0_MOSI_GPIO,
                                       BOARD_SPI0_SCK_GPIO, &g_spi_slave0);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_1_MASTER
  ret = rzv2h_spi_master_bus_initialize(1, BOARD_SPI1_SSL_PIN,
                                         BOARD_SPI1_SSL_USED,
                                         BOARD_SPI1_MISO_GPIO,
                                         BOARD_SPI1_MOSI_GPIO,
                                         BOARD_SPI1_SCK_GPIO, &g_spi1);
  if (ret < 0)
    {
      return ret;
    }
#elif defined(HAVE_RZV2H_SPI_SLAVE) && \
      defined(CONFIG_RZV2H_SPI_CHANNEL_1_SLAVE)
  ret = rzv2h_spi_slave_bus_initialize(1, BOARD_SPI1_SSL_PIN,
                                       BOARD_SPI1_SSL_USED,
                                       BOARD_SPI1_MISO_GPIO,
                                       BOARD_SPI1_MOSI_GPIO,
                                       BOARD_SPI1_SCK_GPIO, &g_spi_slave1);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_SPI_CHANNEL_2_MASTER
  ret = rzv2h_spi_master_bus_initialize(2, BOARD_SPI2_SSL_PIN,
                                         BOARD_SPI2_SSL_USED,
                                         BOARD_SPI2_MISO_GPIO,
                                         BOARD_SPI2_MOSI_GPIO,
                                         BOARD_SPI2_SCK_GPIO, &g_spi2);
  if (ret < 0)
    {
      return ret;
    }
#elif defined(HAVE_RZV2H_SPI_SLAVE) && \
      defined(CONFIG_RZV2H_SPI_CHANNEL_2_SLAVE)
  ret = rzv2h_spi_slave_bus_initialize(2, BOARD_SPI2_SSL_PIN,
                                       BOARD_SPI2_SSL_USED,
                                       BOARD_SPI2_MISO_GPIO,
                                       BOARD_SPI2_MOSI_GPIO,
                                       BOARD_SPI2_SCK_GPIO, &g_spi_slave2);
  if (ret < 0)
    {
      return ret;
    }
#endif

  return OK;
}
