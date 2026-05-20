/****************************************************************************
 * boards/arm/stm32h7/nucleo-h753zi/src/stm32_i2c.c
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
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <stdio.h>

#include "stm32_i2c.h"
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "nucleo-h753zi.h"

#ifdef CONFIG_STM32H7_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_I2C_DEVICES_PER_BUS 16
#define INVALID_I2C_ADDR 0xff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C device registration structure */

struct i2c_device_s
{
  uint8_t addr;           /* I2C slave address (7-bit) */
  uint32_t frequency;     /* Bus frequency for this device */
  char name[16];          /* Device instance name (for logging) */
  bool in_use;            /* true = slot occupied */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C master interfaces (cached after initialization) */

#ifdef CONFIG_STM32H7_I2C1
static struct i2c_master_s *g_i2c1_master = NULL;
static struct i2c_device_s g_i2c1_devices[MAX_I2C_DEVICES_PER_BUS];
#endif

#ifdef CONFIG_STM32H7_I2C2
static struct i2c_master_s *g_i2c2_master = NULL;
static struct i2c_device_s g_i2c2_devices[MAX_I2C_DEVICES_PER_BUS];
#endif

#ifdef CONFIG_STM32H7_I2C3
static struct i2c_master_s *g_i2c3_master = NULL;
static struct i2c_device_s g_i2c3_devices[MAX_I2C_DEVICES_PER_BUS];
#endif

#ifdef CONFIG_STM32H7_I2C4
static struct i2c_master_s *g_i2c4_master = NULL;
static struct i2c_device_s g_i2c4_devices[MAX_I2C_DEVICES_PER_BUS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_i2c_master_ptr
 *
 * Description:
 *   Get pointer to I2C master interface storage for a specific bus.
 *
 * Input Parameters:
 *   i2c_bus - I2C bus number (1-4)
 *
 * Returned Value:
 *   Pointer to I2C master pointer storage, NULL if invalid bus
 *
 ****************************************************************************/

static struct i2c_master_s **get_i2c_master_ptr(int i2c_bus)
{
  switch (i2c_bus)
    {
#ifdef CONFIG_STM32H7_I2C1
      case 1:
        return &g_i2c1_master;
#endif
#ifdef CONFIG_STM32H7_I2C2
      case 2:
        return &g_i2c2_master;
#endif
#ifdef CONFIG_STM32H7_I2C3
      case 3:
        return &g_i2c3_master;
#endif
#ifdef CONFIG_STM32H7_I2C4
      case 4:
        return &g_i2c4_master;
#endif
      default:
        return NULL;
    }
}

/****************************************************************************
 * Name: get_i2c_devices_array
 *
 * Description:
 *   Get I2C devices array for a specific bus.
 *
 * Input Parameters:
 *   i2c_bus - I2C bus number (1-4)
 *
 * Returned Value:
 *   Pointer to I2C devices array, NULL if invalid bus
 *
 ****************************************************************************/

static struct i2c_device_s *get_i2c_devices_array(int i2c_bus)
{
  switch (i2c_bus)
    {
#ifdef CONFIG_STM32H7_I2C1
      case 1:
        return g_i2c1_devices;
#endif
#ifdef CONFIG_STM32H7_I2C2
      case 2:
        return g_i2c2_devices;
#endif
#ifdef CONFIG_STM32H7_I2C3
      case 3:
        return g_i2c3_devices;
#endif
#ifdef CONFIG_STM32H7_I2C4
      case 4:
        return g_i2c4_devices;
#endif
      default:
        return NULL;
    }
}

/****************************************************************************
 * Name: validate_i2c_addr
 *
 * Description:
 *   Validate I2C 7-bit address.
 *   Reserved addresses: 0x00-0x07, 0x78-0x7F
 *
 * Input Parameters:
 *   addr - I2C address to validate
 *
 * Returned Value:
 *   true if valid, false if reserved or out of range
 *
 ****************************************************************************/

static bool validate_i2c_addr(uint8_t addr)
{
  /* I2C 7-bit addressing:
   * 0x00-0x07: Reserved addresses
   * 0x08-0x77: Valid addresses
   * 0x78-0x7F: Reserved addresses
   */

  if (addr < 0x08 || addr > 0x77)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: find_device_slot
 *
 * Description:
 *   Find a free slot in the I2C devices array.
 *
 * Input Parameters:
 *   devices - Pointer to I2C devices array
 *
 * Returned Value:
 *   Index of free slot, -ENOMEM if no slots available
 *
 ****************************************************************************/

static int find_device_slot(struct i2c_device_s *devices)
{
  int i;

  for (i = 0; i < MAX_I2C_DEVICES_PER_BUS; i++)
    {
      if (!devices[i].in_use)
        {
          return i;
        }
    }

  return -ENOMEM;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_initialize
 *
 * Description:
 *   Initialize I2C buses based on Kconfig configuration.
 *   Pins are selected via board.h pinset configuration.
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_i2c_initialize(void)
{
  syslog(LOG_INFO, "Initializing I2C interfaces\n");

#ifdef CONFIG_NUCLEO_H753ZI_I2C1_ENABLE
  g_i2c1_master = stm32_i2cbus_initialize(1);
  if (!g_i2c1_master)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C1\n");
      return -ENODEV;
    }

  memset(g_i2c1_devices, 0, sizeof(g_i2c1_devices));
  syslog(LOG_INFO, "I2C1 initialized successfully\n");
#endif

#ifdef CONFIG_NUCLEO_H753ZI_I2C2_ENABLE
  g_i2c2_master = stm32_i2cbus_initialize(2);
  if (!g_i2c2_master)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C2\n");
      return -ENODEV;
    }

  memset(g_i2c2_devices, 0, sizeof(g_i2c2_devices));
  syslog(LOG_INFO, "I2C2 initialized successfully\n");
#endif

#ifdef CONFIG_NUCLEO_H753ZI_I2C3_ENABLE
  g_i2c3_master = stm32_i2cbus_initialize(3);
  if (!g_i2c3_master)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C3\n");
      return -ENODEV;
    }

  memset(g_i2c3_devices, 0, sizeof(g_i2c3_devices));
  syslog(LOG_INFO, "I2C3 initialized successfully\n");
#endif

#ifdef CONFIG_NUCLEO_H753ZI_I2C4_ENABLE
  g_i2c4_master = stm32_i2cbus_initialize(4);
  if (!g_i2c4_master)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C4\n");
      return -ENODEV;
    }

  memset(g_i2c4_devices, 0, sizeof(g_i2c4_devices));
  syslog(LOG_INFO, "I2C4 initialized successfully\n");
#endif

  syslog(LOG_INFO, "I2C initialization completed\n");
  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_register_device
 *
 * Description:
 *   Register an I2C device with specific address, frequency, and name.
 *   This function tracks device configurations for debugging and
 *   provides validation.
 *
 * Input Parameters:
 *   i2c_bus   - I2C bus number (1-4)
 *   addr      - I2C slave address (7-bit, 0x08-0x77)
 *   frequency - Bus frequency for this device (Hz)
 *   name      - Descriptive name for logging (can be NULL)
 *
 * Returned Value:
 *   OK on success, negative errno on error
 *
 ****************************************************************************/

int stm32_i2c_register_device(int i2c_bus, uint8_t addr,
                               uint32_t frequency, const char *name)
{
  struct i2c_device_s *devices;
  struct i2c_device_s *device;
  int slot;
  int i;

  /* Validate bus */

  devices = get_i2c_devices_array(i2c_bus);
  if (!devices)
    {
      syslog(LOG_ERR, "ERROR: Invalid I2C bus %d\n", i2c_bus);
      return -EINVAL;
    }

  /* Validate address */

  if (!validate_i2c_addr(addr))
    {
      syslog(LOG_ERR,
             "ERROR: Invalid I2C address 0x%02x "
             "(reserved or out of range)\n", addr);
      syslog(LOG_ERR, "       Valid range: 0x08-0x77\n");
      return -EINVAL;
    }

  /* Check for duplicate address */

  for (i = 0; i < MAX_I2C_DEVICES_PER_BUS; i++)
    {
      if (devices[i].in_use && devices[i].addr == addr)
        {
          syslog(LOG_WARNING,
                 "WARNING: Address 0x%02x already registered on I2C%d "
                 "as '%s'\n", addr, i2c_bus, devices[i].name);
          syslog(LOG_WARNING,
                 "         Allowing duplicate (multi-function device?)\n");

          /* Continue anyway - might be multi-function device like LSM303 */
        }
    }

  /* Find free slot */

  slot = find_device_slot(devices);
  if (slot < 0)
    {
      syslog(LOG_ERR,
             "ERROR: No free slots for I2C%d devices "
             "(max %d devices per bus)\n",
             i2c_bus, MAX_I2C_DEVICES_PER_BUS);
      return -ENOMEM;
    }

  /* Register device */

  device = &devices[slot];
  device->addr = addr;
  device->frequency = frequency;
  device->in_use = true;

  if (name != NULL)
    {
      strncpy(device->name, name, sizeof(device->name) - 1);
      device->name[sizeof(device->name) - 1] = '\0';
    }
  else
    {
      snprintf(device->name, sizeof(device->name), "dev_0x%02x", addr);
    }

  syslog(LOG_INFO, "Registered I2C%d device [%d]: '%s' @ 0x%02x, %lu Hz\n",
         i2c_bus, slot, device->name, addr, (unsigned long)frequency);

  return OK;
}

/****************************************************************************
 * Name: stm32_i2c_unregister_device
 *
 * Description:
 *   Unregister an I2C device by address.
 *
 * Input Parameters:
 *   i2c_bus - I2C bus number (1-4)
 *   addr    - I2C slave address to unregister
 *
 * Returned Value:
 *   OK on success, -ENOENT if not found, other negative errno on error
 *
 ****************************************************************************/

int stm32_i2c_unregister_device(int i2c_bus, uint8_t addr)
{
  struct i2c_device_s *devices;
  int i;

  devices = get_i2c_devices_array(i2c_bus);
  if (!devices)
    {
      return -EINVAL;
    }

  /* Find and remove device */

  for (i = 0; i < MAX_I2C_DEVICES_PER_BUS; i++)
    {
      if (devices[i].in_use && devices[i].addr == addr)
        {
          syslog(LOG_INFO,
                 "Unregistered I2C%d device [%d]: '%s' @ 0x%02x\n",
                 i2c_bus, i, devices[i].name, addr);

          memset(&devices[i], 0, sizeof(struct i2c_device_s));
          return OK;
        }
    }

  syslog(LOG_WARNING, "WARNING: Device 0x%02x not found on I2C%d\n",
         addr, i2c_bus);
  return -ENOENT;
}

/****************************************************************************
 * Name: stm32_i2c_get_master
 *
 * Description:
 *   Get I2C master interface for a specific bus.
 *   This is the main function used by sensor drivers to get the I2C
 *   interface they need.
 *
 * Input Parameters:
 *   i2c_bus - I2C bus number (1-4)
 *
 * Returned Value:
 *   Pointer to I2C master interface, NULL if invalid bus or not initialized
 *
 ****************************************************************************/

struct i2c_master_s *stm32_i2c_get_master(int i2c_bus)
{
  struct i2c_master_s **master_ptr;

  master_ptr = get_i2c_master_ptr(i2c_bus);
  if (!master_ptr)
    {
      syslog(LOG_ERR, "ERROR: Invalid I2C bus %d\n", i2c_bus);
      return NULL;
    }

  if (!(*master_ptr))
    {
      syslog(LOG_ERR, "ERROR: I2C%d not initialized\n", i2c_bus);
      return NULL;
    }

  return *master_ptr;
}

/****************************************************************************
 * Name: stm32_i2c_scan_bus
 *
 * Description:
 *   Scan an I2C bus for connected devices.
 *   Useful for debugging and hardware validation.
 *
 * Input Parameters:
 *   i2c_bus - I2C bus number (1-4)
 *
 * Returned Value:
 *   Number of devices found, negative errno on error
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
int stm32_i2c_scan_bus(int i2c_bus)
{
  struct i2c_master_s *i2c;
  struct i2c_msg_s msg;
  uint8_t buffer[1];
  uint8_t addr;
  int found = 0;
  int ret;

  i2c = stm32_i2c_get_master(i2c_bus);
  if (!i2c)
    {
      return -ENODEV;
    }

  syslog(LOG_INFO, "Scanning I2C%d bus (addresses 0x08-0x77)...\n",
         i2c_bus);
  syslog(LOG_INFO,
         "     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

  for (addr = 0x00; addr <= 0x7f; addr++)
    {
      if ((addr & 0x0f) == 0)
        {
          syslog(LOG_INFO, "%02x: ", addr);
        }

      /* Skip reserved addresses */

      if (addr < 0x08 || addr > 0x77)
        {
          syslog(LOG_INFO, "   ");
        }
      else
        {
          /* Try to communicate with device */

          msg.frequency = 100000;  /* 100 kHz for scanning */
          msg.addr = addr;
          msg.flags = 0;
          msg.buffer = buffer;
          msg.length = 0;

          ret = I2C_TRANSFER(i2c, &msg, 1);
          if (ret == OK)
            {
              syslog(LOG_INFO, "%02x ", addr);
              found++;
            }
          else
            {
              syslog(LOG_INFO, "-- ");
            }
        }

      if ((addr & 0x0f) == 0x0f)
        {
          syslog(LOG_INFO, "\n");
        }
    }

  syslog(LOG_INFO, "\nScan complete: %d devices found on I2C%d\n",
         found, i2c_bus);

  return found;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Name: stm32_i2c_list_devices
 *
 * Description:
 *   List all registered I2C devices on a specific bus.
 *   Useful for debugging.
 *
 * Input Parameters:
 *   i2c_bus - I2C bus number (1-4), or 0 for all buses
 *
 * Returned Value:
 *   Number of registered devices
 *
 ****************************************************************************/

int stm32_i2c_list_devices(int i2c_bus)
{
  struct i2c_device_s *devices;
  int total = 0;
  int bus;
  int i;
  int start_bus;
  int end_bus;

  if (i2c_bus == 0)
    {
      /* List all buses */

      start_bus = 1;
      end_bus = 4;
    }
  else
    {
      /* List specific bus */

      start_bus = i2c_bus;
      end_bus = i2c_bus;
    }

  syslog(LOG_INFO, "\nRegistered I2C Devices:\n");
  syslog(LOG_INFO, "%-5s %-4s %-12s %-8s %-10s\n",
         "Bus", "Slot", "Name", "Address", "Frequency");
  syslog(LOG_INFO, "%-5s %-4s %-12s %-8s %-10s\n",
         "---", "----", "------------", "--------", "----------");

  for (bus = start_bus; bus <= end_bus; bus++)
    {
      devices = get_i2c_devices_array(bus);
      if (!devices)
        {
          continue;
        }

      for (i = 0; i < MAX_I2C_DEVICES_PER_BUS; i++)
        {
          if (devices[i].in_use)
            {
              syslog(LOG_INFO,
                     "I2C%-2d %-4d %-12s 0x%02x     %-10lu\n",
                     bus, i, devices[i].name, devices[i].addr,
                     (unsigned long)devices[i].frequency);
              total++;
            }
        }
    }

  if (total == 0)
    {
      syslog(LOG_INFO, "(No devices registered)\n");
    }
  else
    {
      syslog(LOG_INFO, "\nTotal: %d device(s)\n", total);
    }

  return total;
}

#endif /* CONFIG_STM32H7_I2C */
