/****************************************************************************
 * boards/xtensa/esp32s3/common/src/esp32s3_lan9250.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/net/lan9250.h>
#include <arch/board/board.h>

#include "xtensa.h"
#include "esp32s3_gpio.h"
#ifdef CONFIG_LAN9250_SPI
#include "esp32s3_spi.h"
#else
#include "esp32s3_qspi.h"
#endif
#include "hardware/esp32s3_efuse.h"
#include "hardware/esp32s3_gpio_sigmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static int lan9250_attach(const struct lan9250_lower_s *lower,
                          xcpt_t handler, void *arg);
static void lan9250_enable(const struct lan9250_lower_s *lower);
static void lan9250_disable(const struct lan9250_lower_s *lower);
static void lan9250_getmac(const struct lan9250_lower_s *lower,
                           uint8_t *mac);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lan9250_lower_s g_lan9250_lower =
{
  .attach  = lan9250_attach,
  .enable  = lan9250_enable,
  .disable = lan9250_disable,
  .getmac  = lan9250_getmac
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lan9250_attach
 *
 * Description:
 *   Attach LAN9250 interrupt.
 *
 * Input Parameters:
 *   lower   - A reference to the LAN9250 low-level object data.
 *   handler - Interrupt handle
 *   arg     - Interrupt private argument
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lan9250_attach(const struct lan9250_lower_s *lower,
                          xcpt_t handler, void *arg)
{
  int ret;
  int irq = ESP32S3_PIN2IRQ(LAN9250_IRQ);

  ret = irq_attach(irq, handler, arg);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
      return ret;
    }

  ninfo("Attach the interrupt\n");

  return 0;
}

/****************************************************************************
 * Name: lan9250_enable
 *
 * Description:
 *   Enable LAN9250 interrupt.
 *
 * Input Parameters:
 *   lower - A reference to the LAN9250 low-level object data.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_enable(const struct lan9250_lower_s *lower)
{
  int irq = ESP32S3_PIN2IRQ(LAN9250_IRQ);

  /* Configure the interrupt for rising and falling edges */

  esp32s3_gpioirqenable(irq, GPIO_INTR_LOW_LEVEL);
  ninfo("Enable the interrupt\n");
}

/****************************************************************************
 * Name: lan9250_disable
 *
 * Description:
 *   Disable LAN9250 interrupt.
 *
 * Input Parameters:
 *   lower - A reference to the LAN9250 low-level object data.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_disable(const struct lan9250_lower_s *lower)
{
  int irq = ESP32S3_PIN2IRQ(LAN9250_IRQ);

  ninfo("Disable the interrupt\n");
  esp32s3_gpioirqdisable(irq);
}

/****************************************************************************
 * Name: lan9250_getmac
 *
 * Description:
 *   Retrieves the ESP32-S3 MAC address to be set in the LAN9250 driver.
 *
 * Input Parameters:
 *   lower - A reference to the LAN9250 low-level object data.
 *   mac   - A pointer to a buffer where the MAC address will be stored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lan9250_getmac(const struct lan9250_lower_s *lower, uint8_t *mac)
{
  uint32_t regval[2];
  uint8_t *data = (uint8_t *)regval;

  regval[0] = getreg32(EFUSE_RD_MAC_SPI_SYS_0_REG);
  regval[1] = getreg32(EFUSE_RD_MAC_SPI_SYS_1_REG);

  for (int i = 0; i < 6; i++)
    {
      mac[i] = data[i];
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_lan9250_initialize
 *
 * Description:
 *   This function is called by platform-specific setup logic to initialize
 *   the LAN9250 device. This function will register the driver
 *   as a network device.
 *
 * Input Parameters:
 *   port - The SPI port used for the device
 *
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int esp32s3_lan9250_initialize(int port)
{
  int ret;
#ifdef CONFIG_LAN9250_SPI
  struct spi_dev_s *dev;
#else
  struct qspi_dev_s *dev;
#endif

  esp32s3_configgpio(LAN9250_IRQ, INPUT_FUNCTION_2 | PULLUP);
  esp32s3_configgpio(LAN9250_RST, OUTPUT_FUNCTION_2 | PULLUP);

#ifdef CONFIG_LAN9250_SPI
  dev = esp32s3_spibus_initialize(port);
  if (!dev)
    {
      nerr("ERROR: Failed to initialize SPI port %d\n", port);
      return -ENODEV;
    }
#else
  dev = esp32s3_qspibus_initialize(port);
  if (!dev)
    {
      nerr("ERROR: Failed to initialize QSPI port %d\n", port);
      return -ENODEV;
    }
#endif

  ret = lan9250_initialize(dev, &g_lan9250_lower);
  if (ret != 0)
    {
      nerr("ERROR: Failed to initialize LAN9250 ret=%d\n", ret);
      return ret;
    }

  return 0;
}

