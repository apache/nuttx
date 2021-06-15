/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_boot.c
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
#include <arch/board/board.h>
#include <syslog.h>

#include "stm32_butterfly2.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   Initializes low level pins for the drivers.
 ****************************************************************************/

void stm32_boardinitialize(void)
{
  stm32_led_initialize();
  stm32_spidev_initialize();
  stm32_usb_initialize();
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Initializes upper half drivers with board specific settings
 *
 * Returned Value:
 *   0 on success or errno value of failed init function.
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  int ret = 0;

#ifdef CONFIG_MMCSD
  ret = stm32_mmcsd_initialize(CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_USBHOST
  ret = stm32_usbhost_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return ret;
}
