/****************************************************************************
 * boards/arm/stm32h7/nucleo-h753zi/src/stm32_spi.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
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
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "nucleo-h753zi.h"
#include <nuttx/spi/spi_transfer.h>

#ifdef CONFIG_STM32H7_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_CS_DEVICES_PER_SPI 16
#define INVALID_CS_PIN 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CS pin registration structure */

struct spi_cs_device_s
{
  uint32_t gpio_config;    /* GPIO configuration for CS pin */
  bool active_low;         /* true = active low, false = active high */
  bool in_use;             /* true = slot occupied */
};

/* DC pin registration structure */

#ifdef CONFIG_SPI_CMDDATA
struct spi_dc_device_s
{
  uint32_t gpio_config;  /* GPIO configuration for DC pin */
  bool in_use;           /* true = slot occupied */
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* CS device registrations for each SPI bus */

#ifdef CONFIG_STM32H7_SPI1
static struct spi_cs_device_s g_spi1_cs_devices[MAX_CS_DEVICES_PER_SPI];
#ifdef CONFIG_SPI_CMDDATA
static struct spi_dc_device_s g_spi1_dc_devices[MAX_CS_DEVICES_PER_SPI];
#endif
#endif

#ifdef CONFIG_STM32H7_SPI2
static struct spi_cs_device_s g_spi2_cs_devices[MAX_CS_DEVICES_PER_SPI];
#ifdef CONFIG_SPI_CMDDATA
static struct spi_dc_device_s g_spi2_dc_devices[MAX_CS_DEVICES_PER_SPI];
#endif
#endif

#ifdef CONFIG_STM32H7_SPI3
static struct spi_cs_device_s g_spi3_cs_devices[MAX_CS_DEVICES_PER_SPI];
#ifdef CONFIG_SPI_CMDDATA
static struct spi_dc_device_s g_spi3_dc_devices[MAX_CS_DEVICES_PER_SPI];
#endif
#endif

#ifdef CONFIG_STM32H7_SPI4
static struct spi_cs_device_s g_spi4_cs_devices[MAX_CS_DEVICES_PER_SPI];
#ifdef CONFIG_SPI_CMDDATA
static struct spi_dc_device_s g_spi4_dc_devices[MAX_CS_DEVICES_PER_SPI];
#endif
#endif

#ifdef CONFIG_STM32H7_SPI5
static struct spi_cs_device_s g_spi5_cs_devices[MAX_CS_DEVICES_PER_SPI];
#ifdef CONFIG_SPI_CMDDATA
static struct spi_dc_device_s g_spi5_dc_devices[MAX_CS_DEVICES_PER_SPI];
#endif
#endif

#ifdef CONFIG_STM32H7_SPI6
static struct spi_cs_device_s g_spi6_cs_devices[MAX_CS_DEVICES_PER_SPI];
#ifdef CONFIG_SPI_CMDDATA
static struct spi_dc_device_s g_spi6_dc_devices[MAX_CS_DEVICES_PER_SPI];
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_gpio_pin
 *
 * Description:
 *   Parse GPIO pin string like "PA0" into STM32 GPIO configuration.
 *
 * Input Parameters:
 *   pin_str - GPIO pin string (e.g., "PA0", "PF15", "PC13")
 *   error   - Pointer to error code storage
 *
 * Returned Value:
 *   STM32 GPIO configuration value on success, 0 on error
 *
 ****************************************************************************/

static uint32_t parse_gpio_pin(FAR const char *pin_str, FAR int *error)
{
  size_t len;
  char port;
  FAR const char *pin_num_str;
  FAR char *endptr;
  long pin_num;
  uint32_t port_base;
  uint32_t gpio_pin;

  *error = 0;

  if (pin_str == NULL)
    {
      *error = -EINVAL;
      return 0;
    }

  /* Remove leading/trailing spaces */

  while (*pin_str == ' ' || *pin_str == '\t')
    {
      pin_str++;
    }

  len = strlen(pin_str);
  if (len < 3 || len > 4)
    {
      *error = -EINVAL;
      return 0;
    }

  if (pin_str[0] != 'P')
    {
      *error = -EINVAL;
      return 0;
    }

  port = pin_str[1];
  if (port < 'A' || port > 'H')
    {
      *error = -EINVAL;
      return 0;
    }

  pin_num_str = &pin_str[2];
  pin_num = strtol(pin_num_str, &endptr, 10);
  if (*endptr != '\0' || pin_num < 0 || pin_num > 15)
    {
      *error = -EINVAL;
      return 0;
    }

  /* Map port letter to STM32 port base */

  switch (port)
    {
      case 'A':
        port_base = GPIO_PORTA; break;
      case 'B':
        port_base = GPIO_PORTB; break;
      case 'C':
        port_base = GPIO_PORTC; break;
      case 'D':
        port_base = GPIO_PORTD; break;
      case 'E':
        port_base = GPIO_PORTE; break;
      case 'F':
        port_base = GPIO_PORTF; break;
      case 'G':
        port_base = GPIO_PORTG; break;
      case 'H':
        port_base = GPIO_PORTH; break;
      default:
        *error = -EINVAL;
        return 0;
    }

  /* Use correct STM32 GPIO pin macros */

  switch (pin_num)
    {
      case 0:
        gpio_pin = GPIO_PIN0;  break;
      case 1:
        gpio_pin = GPIO_PIN1;  break;
      case 2:
        gpio_pin = GPIO_PIN2;  break;
      case 3:
        gpio_pin = GPIO_PIN3;  break;
      case 4:
        gpio_pin = GPIO_PIN4;  break;
      case 5:
        gpio_pin = GPIO_PIN5;  break;
      case 6:
        gpio_pin = GPIO_PIN6;  break;
      case 7:
        gpio_pin = GPIO_PIN7;  break;
      case 8:
        gpio_pin = GPIO_PIN8;  break;
      case 9:
        gpio_pin = GPIO_PIN9;  break;
      case 10:
        gpio_pin = GPIO_PIN10; break;
      case 11:
        gpio_pin = GPIO_PIN11; break;
      case 12:
        gpio_pin = GPIO_PIN12; break;
      case 13:
        gpio_pin = GPIO_PIN13; break;
      case 14:
        gpio_pin = GPIO_PIN14; break;
      case 15:
        gpio_pin = GPIO_PIN15; break;
      default:
        *error = -EINVAL;
        return 0;
    }

  return (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_SPEED_50MHz | GPIO_FLOAT |
          port_base | gpio_pin);
}

/****************************************************************************
 * Name: get_cs_devices_array
 *
 * Description:
 *   Get CS devices array for a specific SPI bus.
 *
 * Input Parameters:
 *   spi_bus - SPI bus number (1-6)
 *
 * Returned Value:
 *   Pointer to CS devices array, NULL if invalid bus
 *
 ****************************************************************************/

static struct spi_cs_device_s *get_cs_devices_array(int spi_bus)
{
  switch (spi_bus)
    {
#ifdef CONFIG_STM32H7_SPI1
      case 1:
        return g_spi1_cs_devices;
#endif
#ifdef CONFIG_STM32H7_SPI2
      case 2:
        return g_spi2_cs_devices;
#endif
#ifdef CONFIG_STM32H7_SPI3
      case 3:
        return g_spi3_cs_devices;
#endif
#ifdef CONFIG_STM32H7_SPI4
      case 4:
        return g_spi4_cs_devices;
#endif
#ifdef CONFIG_STM32H7_SPI5
      case 5:
        return g_spi5_cs_devices;
#endif
#ifdef CONFIG_STM32H7_SPI6
      case 6:
        return g_spi6_cs_devices;
#endif
      default:
        return NULL;
    }
}

/****************************************************************************
 * Name: get_dc_devices_array
 *
 * Description:
 *   Get DC devices array for a specific SPI bus.
 *
 * Input Parameters:
 *   spi_bus - SPI bus number (1-6)
 *
 * Returned Value:
 *   Pointer to DC devices array, NULL if invalid bus
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static struct spi_dc_device_s *get_dc_devices_array(int spi_bus)
{
  switch (spi_bus)
    {
#ifdef CONFIG_STM32H7_SPI1
      case 1:
        return g_spi1_dc_devices;
#endif
#ifdef CONFIG_STM32H7_SPI2
      case 2:
        return g_spi2_dc_devices;
#endif
#ifdef CONFIG_STM32H7_SPI3
      case 3:
        return g_spi3_dc_devices;
#endif
#ifdef CONFIG_STM32H7_SPI4
      case 4:
        return g_spi4_dc_devices;
#endif
#ifdef CONFIG_STM32H7_SPI5
      case 5:
        return g_spi5_dc_devices;
#endif
#ifdef CONFIG_STM32H7_SPI6
      case 6:
        return g_spi6_dc_devices;
#endif
      default:
        return NULL;
    }
}
#endif

/****************************************************************************
 * Name: spi_cs_control
 *
 * Description:
 *   Control SPI chip select pin based on registered device.
 *
 * Input Parameters:
 *   spi_bus  - SPI bus number (1-6)
 *   devid    - Device ID within the bus
 *   selected - Select (true) or deselect (false)
 *
 ****************************************************************************/

static void spi_cs_control(int spi_bus, uint32_t devid, bool selected)
{
  struct spi_cs_device_s *cs_devices;
  struct spi_cs_device_s *device;
  bool pin_state;
  uint32_t actual_devid = devid;

  cs_devices = get_cs_devices_array(spi_bus);
  if (cs_devices == NULL)
    {
      spierr("ERROR: Invalid SPI bus %d\n", spi_bus);
      return;
    }

  /* Handle SPIDEV_* types */

  if ((devid & 0xffff0000) != 0)
    {
      actual_devid = (devid & 0x0000ffff);
      spiinfo("Detected SPIDEV type 0x%04lX, using index %lu for SPI%d\n",
              (unsigned long)(devid >> 16), (unsigned long)actual_devid,
              spi_bus);
    }
  else if (devid >= MAX_CS_DEVICES_PER_SPI)
    {
      spiwarn("WARNING: Device ID %lu >= maximum %d for SPI%d, "
              "trying fallback to ID 0\n",
              (unsigned long)devid, MAX_CS_DEVICES_PER_SPI, spi_bus);

      if (cs_devices[0].in_use)
        {
          actual_devid = 0;
          spiinfo("SUCCESS: Using fallback device ID 0 for invalid ID %lu\n",
                  (unsigned long)devid);
        }
      else
        {
          spierr("ERROR: Device ID %lu >= maximum %d and no fallback "
                 "available for SPI%d\n",
                 (unsigned long)devid, MAX_CS_DEVICES_PER_SPI, spi_bus);
          return;
        }
    }

  device = &cs_devices[actual_devid];
  if (!device->in_use)
    {
      spierr("ERROR: Device ID %lu (actual %lu) not registered for SPI%d\n",
             (unsigned long)devid, (unsigned long)actual_devid, spi_bus);
      return;
    }

  /* Calculate pin state */

  if (device->active_low)
    {
      pin_state = !selected;
    }
  else
    {
      pin_state = selected;
    }

  stm32_gpiowrite(device->gpio_config, pin_state);

  spiinfo("SPI%d CS%lu->%lu: %s (pin %s)\n",
          spi_bus, (unsigned long)devid, (unsigned long)actual_devid,
          selected ? "SELECT" : "DESELECT",
          pin_state ? "HIGH" : "LOW");
}

/****************************************************************************
 * Name: spi_dc_control
 *
 * Description:
 *   Control SPI DC (Data/Command) pin for registered device.
 *
 * Input Parameters:
 *   spi_bus - SPI bus number (1-6)
 *   devid   - Device ID within the bus
 *   cmd     - true = command mode, false = data mode
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static void spi_dc_control(int spi_bus, uint32_t devid, bool cmd)
{
  struct spi_dc_device_s *dc_devices;
  struct spi_dc_device_s *device;
  uint32_t actual_devid = devid;

  dc_devices = get_dc_devices_array(spi_bus);
  if (dc_devices == NULL)
    {
      spierr("ERROR: Invalid SPI bus %d for DC control\n", spi_bus);
      return;
    }

  /* Handle SPIDEV_* types */

  if ((devid & 0xffff0000) != 0)
    {
      actual_devid = (devid & 0x0000ffff);
    }
  else if (devid >= MAX_CS_DEVICES_PER_SPI)
    {
      spierr("ERROR: Device ID %lu >= maximum %d for DC control on SPI%d\n",
             (unsigned long)devid, MAX_CS_DEVICES_PER_SPI, spi_bus);
      return;
    }

  device = &dc_devices[actual_devid];
  if (!device->in_use)
    {
      spierr("ERROR: DC pin not registered for SPI%d device %lu\n",
             spi_bus, (unsigned long)actual_devid);
      return;
    }

  /* Set DC pin: LOW for command, HIGH for data
   * This is the standard convention for SPI displays
   */

  stm32_gpiowrite(device->gpio_config, !cmd);

  /* DEBUG SESSION
   * spiinfo("SPI%d DC%lu: %s (pin %s)\n",
   *       spi_bus, (unsigned long)actual_devid,
   *       cmd ? "COMMAND" : "DATA",
   *       cmd ? "LOW" : "HIGH");
   *
   * syslog(LOG_INFO, "SPI%d DC%lu: %s (pin %s) [GPIO=0x%08lx]\n",
   *       spi_bus, (unsigned long)actual_devid,
   *       cmd ? "COMMAND" : "DATA",
   *       cmd ? "LOW" : "HIGH",
   *      (unsigned long)device->gpio_config);
   */
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spi_initialize
 *
 * Description:
 *   Initialize SPI interfaces.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_spi_initialize(void)
{
  spiinfo("stm32_spi_initialize: Initializing SPI interfaces\n");

#ifdef CONFIG_STM32H7_SPI1
  memset(g_spi1_cs_devices, 0, sizeof(g_spi1_cs_devices));
#ifdef CONFIG_SPI_CMDDATA
  memset(g_spi1_dc_devices, 0, sizeof(g_spi1_dc_devices));
#endif
  spiinfo("SPI1 CS/DC registry initialized\n");
#endif

#ifdef CONFIG_STM32H7_SPI2
  memset(g_spi2_cs_devices, 0, sizeof(g_spi2_cs_devices));
#ifdef CONFIG_SPI_CMDDATA
  memset(g_spi2_dc_devices, 0, sizeof(g_spi2_dc_devices));
#endif
  spiinfo("SPI2 CS/DC registry initialized\n");
#endif

#ifdef CONFIG_STM32H7_SPI3
  memset(g_spi3_cs_devices, 0, sizeof(g_spi3_cs_devices));
#ifdef CONFIG_SPI_CMDDATA
  memset(g_spi3_dc_devices, 0, sizeof(g_spi3_dc_devices));
#endif
  spiinfo("SPI3 CS/DC registry initialized\n");
#endif

#ifdef CONFIG_STM32H7_SPI4
  memset(g_spi4_cs_devices, 0, sizeof(g_spi4_cs_devices));
#ifdef CONFIG_SPI_CMDDATA
  memset(g_spi4_dc_devices, 0, sizeof(g_spi4_dc_devices));
#endif
  spiinfo("SPI4 CS/DC registry initialized\n");
#endif

#ifdef CONFIG_STM32H7_SPI5
  memset(g_spi5_cs_devices, 0, sizeof(g_spi5_cs_devices));
#ifdef CONFIG_SPI_CMDDATA
  memset(g_spi5_dc_devices, 0, sizeof(g_spi5_dc_devices));
#endif
  spiinfo("SPI5 CS/DC registry initialized\n");
#endif

#ifdef CONFIG_STM32H7_SPI6
  memset(g_spi6_cs_devices, 0, sizeof(g_spi6_cs_devices));
#ifdef CONFIG_SPI_CMDDATA
  memset(g_spi6_dc_devices, 0, sizeof(g_spi6_dc_devices));
#endif
  spiinfo("SPI6 CS/DC registry initialized\n");
#endif

  spiinfo("SPI initialization completed\n");
  return OK;
}

/****************************************************************************
 * Name: stm32_spi_register_cs_device
 *
 * Description:
 *   Register a CS device for a specific SPI bus and device ID.
 *
 ****************************************************************************/

int stm32_spi_register_cs_device(int spi_bus, uint32_t devid,
                                  const char *cs_pin, bool active_low)
{
  struct spi_cs_device_s *cs_devices;
  struct spi_cs_device_s *device;
  uint32_t gpio_config;
  int error;
  int ret;

  cs_devices = get_cs_devices_array(spi_bus);
  if (cs_devices == NULL)
    {
      spierr("ERROR: Invalid SPI bus %d\n", spi_bus);
      return -EINVAL;
    }

  if (devid >= MAX_CS_DEVICES_PER_SPI)
    {
      spierr("ERROR: Device ID %lu >= maximum %d\n",
             (unsigned long)devid, MAX_CS_DEVICES_PER_SPI);
      return -EINVAL;
    }

  device = &cs_devices[devid];
  if (device->in_use)
    {
      spierr("ERROR: Device ID %lu already registered for SPI%d\n",
             (unsigned long)devid, spi_bus);
      return -EBUSY;
    }

  gpio_config = parse_gpio_pin(cs_pin, &error);
  if (error != 0)
    {
      spierr("ERROR: Invalid CS pin '%s': %d\n", cs_pin, error);
      return error;
    }

  ret = stm32_configgpio(gpio_config);
  if (ret < 0)
    {
      spierr("ERROR: Failed to configure CS pin %s: %d\n", cs_pin, ret);
      return ret;
    }

  stm32_gpiowrite(gpio_config, active_low ? true : false);

  device->gpio_config = gpio_config;
  device->active_low = active_low;
  device->in_use = true;

  spiinfo("Registered SPI%d device %lu: pin %s (%s)\n",
          spi_bus, (unsigned long)devid, cs_pin,
          active_low ? "active_low" : "active_high");

  return OK;
}

/****************************************************************************
 * Name: stm32_spi_register_dc_pin
 *
 * Description:
 *   Register a DC (Data/Command) pin for a specific SPI device.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
int stm32_spi_register_dc_pin(int spi_bus, uint32_t devid,
                               const char *dc_pin)
{
  struct spi_dc_device_s *dc_devices;
  struct spi_dc_device_s *device;
  uint32_t gpio_config;
  int error;
  int ret;

  dc_devices = get_dc_devices_array(spi_bus);
  if (dc_devices == NULL)
    {
      spierr("ERROR: Invalid SPI bus %d\n", spi_bus);
      return -EINVAL;
    }

  if (devid >= MAX_CS_DEVICES_PER_SPI)
    {
      spierr("ERROR: Device ID %lu >= maximum %d\n",
             (unsigned long)devid, MAX_CS_DEVICES_PER_SPI);
      return -EINVAL;
    }

  device = &dc_devices[devid];
  if (device->in_use)
    {
      spierr("ERROR: DC pin already registered for SPI%d device %lu\n",
             spi_bus, (unsigned long)devid);
      return -EBUSY;
    }

  gpio_config = parse_gpio_pin(dc_pin, &error);
  if (error != 0)
    {
      spierr("ERROR: Invalid DC pin '%s': %d\n", dc_pin, error);
      return error;
    }

  ret = stm32_configgpio(gpio_config);
  if (ret < 0)
    {
      spierr("ERROR: Failed to configure DC pin %s: %d\n", dc_pin, ret);
      return ret;
    }

  /* Initialize DC pin to data mode (HIGH) */

  stm32_gpiowrite(gpio_config, true);

  device->gpio_config = gpio_config;
  device->in_use = true;

  spiinfo("Registered SPI%d device %lu DC pin: %s\n",
          spi_bus, (unsigned long)devid, dc_pin);

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_spi_unregister_cs_device
 *
 * Description:
 *   Unregister a CS device.
 *
 ****************************************************************************/

int stm32_spi_unregister_cs_device(int spi_bus, uint32_t devid)
{
  struct spi_cs_device_s *cs_devices;
  struct spi_cs_device_s *device;

  cs_devices = get_cs_devices_array(spi_bus);
  if (cs_devices == NULL)
    {
      return -EINVAL;
    }

  if (devid >= MAX_CS_DEVICES_PER_SPI)
    {
      return -EINVAL;
    }

  device = &cs_devices[devid];
  if (!device->in_use)
    {
      return -ENOENT;
    }

  device->gpio_config = INVALID_CS_PIN;
  device->active_low = false;
  device->in_use = false;

  spiinfo("Unregistered SPI%d device %lu\n", spi_bus,
          (unsigned long)devid);
  return OK;
}

/****************************************************************************
 * Name: stm32_spidev_register_all
 *
 * Description:
 *   Register SPI devices for user access.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DRIVER
int stm32_spidev_register_all(void)
{
  int ret = OK;

#ifdef CONFIG_STM32H7_SPI1
  FAR struct spi_dev_s *spi1;

  spi1 = stm32_spibus_initialize(1);
  if (spi1)
    {
      ret = spi_register(spi1, 1);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register SPI1: %d\n", ret);
        }
      else
        {
          syslog(LOG_INFO, "SPI1 registered as /dev/spi1\n");
        }
    }
#endif

#ifdef CONFIG_STM32H7_SPI2
  FAR struct spi_dev_s *spi2;

  spi2 = stm32_spibus_initialize(2);
  if (spi2)
    {
      ret = spi_register(spi2, 2);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register SPI2: %d\n", ret);
        }
      else
        {
          syslog(LOG_INFO, "SPI2 registered as /dev/spi2\n");
        }
    }
#endif

#ifdef CONFIG_STM32H7_SPI3
  FAR struct spi_dev_s *spi3;

  spi3 = stm32_spibus_initialize(3);
  if (spi3)
    {
      ret = spi_register(spi3, 3);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register SPI3: %d\n", ret);
        }
      else
        {
          syslog(LOG_INFO, "SPI3 registered as /dev/spi3\n");
        }
    }
#endif

#ifdef CONFIG_STM32H7_SPI4
  FAR struct spi_dev_s *spi4;

  spi4 = stm32_spibus_initialize(4);
  if (spi4)
    {
      ret = spi_register(spi4, 4);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register SPI4: %d\n", ret);
        }
      else
        {
          syslog(LOG_INFO, "SPI4 registered as /dev/spi4\n");
        }
    }
#endif

#ifdef CONFIG_STM32H7_SPI5
  FAR struct spi_dev_s *spi5;

  spi5 = stm32_spibus_initialize(5);
  if (spi5)
    {
      ret = spi_register(spi5, 5);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register SPI5: %d\n", ret);
        }
      else
        {
          syslog(LOG_INFO, "SPI5 registered as /dev/spi5\n");
        }
    }
#endif

#ifdef CONFIG_STM32H7_SPI6
  FAR struct spi_dev_s *spi6;

  spi6 = stm32_spibus_initialize(6);
  if (spi6)
    {
      ret = spi_register(spi6, 6);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register SPI6: %d\n", ret);
        }
      else
        {
          syslog(LOG_INFO, "SPI6 registered as /dev/spi6\n");
        }
    }
#endif

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_spi1/2/3/4/5/6_select
 *
 * Description:
 *   SPI select functions - called by STM32 SPI driver
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("SPI1 CS: devid=%lu, %s\n",
          (unsigned long)devid, selected ? "SELECT" : "DESELECT");
  spi_cs_control(1, devid, selected);
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

#ifdef CONFIG_SPI_CMDDATA
int stm32_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  spiinfo("SPI1 CMDDATA: devid=%lu, %s\n",
          (unsigned long)devid, cmd ? "COMMAND" : "DATA");
  spi_dc_control(1, devid, cmd);
  return OK;
}
#endif
#endif

#ifdef CONFIG_STM32H7_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("SPI2 CS: devid=%lu, %s\n",
          (unsigned long)devid, selected ? "SELECT" : "DESELECT");
  spi_cs_control(2, devid, selected);
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

#ifdef CONFIG_SPI_CMDDATA
int stm32_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  spiinfo("SPI2 CMDDATA: devid=%lu, %s\n",
          (unsigned long)devid, cmd ? "COMMAND" : "DATA");
  spi_dc_control(2, devid, cmd);
  return OK;
}
#endif
#endif

#ifdef CONFIG_STM32H7_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("SPI3 CS: devid=%lu, %s\n",
          (unsigned long)devid, selected ? "SELECT" : "DESELECT");
  spi_cs_control(3, devid, selected);
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

#ifdef CONFIG_SPI_CMDDATA
int stm32_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  spiinfo("SPI3 CMDDATA: devid=%lu, %s\n",
          (unsigned long)devid, cmd ? "COMMAND" : "DATA");
  spi_dc_control(3, devid, cmd);
  return OK;
}
#endif
#endif

#ifdef CONFIG_STM32H7_SPI4
void stm32_spi4select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("SPI4 CS: devid=%lu, %s\n",
          (unsigned long)devid, selected ? "SELECT" : "DESELECT");
  spi_cs_control(4, devid, selected);
}

uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

#ifdef CONFIG_SPI_CMDDATA
int stm32_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  spiinfo("SPI4 CMDDATA: devid=%lu, %s\n",
          (unsigned long)devid, cmd ? "COMMAND" : "DATA");
  spi_dc_control(4, devid, cmd);
  return OK;
}
#endif
#endif

#ifdef CONFIG_STM32H7_SPI5
void stm32_spi5select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("SPI5 CS: devid=%lu, %s\n",
          (unsigned long)devid, selected ? "SELECT" : "DESELECT");
  spi_cs_control(5, devid, selected);
}

uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

#ifdef CONFIG_SPI_CMDDATA
int stm32_spi5cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  spiinfo("SPI5 CMDDATA: devid=%lu, %s\n",
          (unsigned long)devid, cmd ? "COMMAND" : "DATA");
  spi_dc_control(5, devid, cmd);
  return OK;
}
#endif
#endif

#ifdef CONFIG_STM32H7_SPI6
void stm32_spi6select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("SPI6 CS: devid=%lu, %s\n",
          (unsigned long)devid, selected ? "SELECT" : "DESELECT");
  spi_cs_control(6, devid, selected);
}

uint8_t stm32_spi6status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

#ifdef CONFIG_SPI_CMDDATA
int stm32_spi6cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  spiinfo("SPI6 CMDDATA: devid=%lu, %s\n",
          (unsigned long)devid, cmd ? "COMMAND" : "DATA");
  spi_dc_control(6, devid, cmd);
  return OK;
}
#endif
#endif

#endif /* CONFIG_STM32H7_SPI */
