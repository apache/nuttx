/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_ds18b20.c
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

#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/1wire/1wire_master.h>
#include <nuttx/sensors/ds18b20.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_1wire.h"
#include "stm32f103_minimum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef BOARD_DS18B20_NBUS
# error "Definition BOARD_DS18B20_NBUS is missing"
#endif
#ifndef BOARD_DS18B20_NSLAVES
# error "Definition BOARD_DS18B20_NSLAVES is missing"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Callback handling discovered devices */

struct ds18b20_device_s
{
  uint64_t *romcode;
  int      ndevices;
  int      discovered;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ds18b20_cb_search
 *
 * Description:
 *   Call back function for searching devices on the bus.
 *
 * Input Parameters:
 *   romcode - Unique romcode of a device with alarm flag set
 *   arg     - Pointer to struct onewire_device_s
 *
 ****************************************************************************/

static void stm32_ds18b20_cb_search(int family, uint64_t romcode,
                                    void *arg)
{
  struct ds18b20_device_s *priv = (struct ds18b20_device_s *)arg;

  if (priv->discovered < priv->ndevices - 1)
    {
      priv->romcode[priv->discovered] = romcode;
      priv->discovered++;
    }
}

/****************************************************************************
 * Name: stm32_ds18b20register
 *
 * Description:
 *   Discover DS18B20 sensors on the bus and register.
 *
 * Parameter:
 *   onewire   - Pointer to allocated onewire lower galf instance
 *   maxslaves - Number of expected devices on the bus
 *   devno     - Device number
 *
 * Return:
 *   OK on success
 *
 ****************************************************************************/

static int stm32_ds18b20register(struct onewire_master_s *onewire,
                                 int maxslaves, int devno)
{
  int n;
  int ret;
  uint64_t romcode[maxslaves];
  struct ds18b20_device_s device;
  device.romcode     = romcode;
  device.ndevices    = maxslaves;
  device.discovered  = 0;

  ret = onewire_search(onewire, DS18B20_DEVICE_FAMILY, false,
                       stm32_ds18b20_cb_search, &device);
  if (ret < 0)
    {
      ierr("ERROR: Discover DS18B20 devices failed: %d\n", errno);
      return ret;
    }

  DEBUGASSERT(ret > maxslaves);

  for (n = 0; n < device.discovered; n++)
    {
      ret = ds18b20_register(devno, onewire, romcode[n]);
      if (ret < 0)
        {
          ierr("ERROR: Failed to register DS18B20 device with romcode %lld: \
          %d\n", romcode[n], errno);
        }
      else
        {
          devno++;
        }
    }

  return devno;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ds18b20initialize
 *
 * Description:
 *   Function used to initialize DS18B20 snesors on a 1wire bus
 *
 * Parameter:
 *   devno   - First character device number
 *
 * Return
 *   Error or number of device that have been successfully registered.
 *
 ****************************************************************************/

int stm32_ds18b20initialize(int devno)
{
  struct onewire_dev_s *dev;
  struct onewire_master_s *onewire;

  dev = stm32_1wireinitialize(BOARD_DS18B20_NBUS);
  if (!dev)
    {
      ierr("ERROR: Failed to initialize arch specific 1Wire bus %d;: %d\n",
           BOARD_DS18B20_NSLAVES, errno);
      return -ENODEV;
    }

  onewire = onewire_initialize(dev, BOARD_DS18B20_NSLAVES);
  if (!onewire)
    {
      ierr("ERROR: Failed to initialize 1Wire bus %d;: %d\n",
           BOARD_DS18B20_NBUS, errno);
      return -ENODEV;
    }

  return stm32_ds18b20register(onewire, BOARD_DS18B20_NSLAVES, devno);
}
