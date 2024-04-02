/****************************************************************************
 * boards/arm64/imx9/imx93-evk/src/imx9_spi.c
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

#include <debug.h>
#include <errno.h>
#include <sys/types.h>

#include <nuttx/spi/spi_transfer.h>
#include <arch/board/board.h>

#include "imx9_gpio.h"
#include "imx9_lpspi.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI6
static struct spi_dev_s *g_spi6;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_IMX9_LPSPI

/****************************************************************************
 * Name: imx9_lpspix_select
 *
 * Description:
 *   Enable/disable the SPI chip select.  The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *   Required.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx9_lpspi_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
#ifdef CONFIG_IMX9_LPSPI6
  if (dev == g_spi6)
    {
      imx9_gpio_write(GPIO_LPSPI6_CS, !selected);
    }
#endif
}

/****************************************************************************
 * Name: imx9_lpspix_status
 *
 * Description:
 *   Get SPI/MMC status.  Optional.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/

uint8_t imx9_lpspi_status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

/****************************************************************************
 * Name: imx9_lpspixcmddata
 *
 * Description:
 *   Some devices require an additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

int imx9_lpspi_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

/****************************************************************************
 * Name: board_spi_initialize
 *
 * Description:
 *   Initialize and register SPI driver for the defined SPI ports.
 *
 ****************************************************************************/

int imx9_spi_initialize(void)
{
  int ret = OK;

#if defined(CONFIG_IMX9_LPSPI)
  /* Initialize SPI device */

  g_spi6 = imx9_lpspibus_initialize(6);
  if (g_spi6 == NULL)
    {
      spierr("Failed to initialize SPI6\n");
      return -ENODEV;
    }
#endif /* CONFIG_MPFS_SPI0 */

#ifdef CONFIG_SPI_DRIVER
  ret = spi_register(g_spi6, 0);
  if (ret < 0)
    {
      spierr("Failed to register /dev/spi0: %d\n", ret);
    }
#endif /* CONFIG_SPI_DRIVER */

  return OK;
}
