/****************************************************************************
 * boards/arm/nrf91/thingy91/src/nrf91_bringup.c
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

#include <errno.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_NRF91_MODEM
#  include "nrf91_modem.h"
#endif

#ifdef CONFIG_NRF91_MODEM_AT
#  include "nrf91_modem_at.h"
#endif

#ifdef CONFIG_NRF91_PROGMEM
#  include "nrf91_progmem.h"
#endif

#ifdef CONFIG_TIMER
#  include "nrf91_timer.h"
#endif

#include "thingy91.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF91_TIMER (0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int nrf91_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, NRF91_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_TIMER) && defined(CONFIG_NRF91_TIMER)
  /* Configure TIMER driver */

  ret = nrf91_timer_driver_setup("/dev/timer0", NRF91_TIMER);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_NRF91_MODEM
  /* Initialize modem */

  ret = nrf91_modem_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize modem%d\n",  ret);
    }
#endif

#ifdef CONFIG_NRF91_MODEM_AT
  /* Initialize modem AT interface */

  ret = nrf91_at_register("/dev/modem");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize AT interface%d\n",  ret);
    }
#endif

#ifdef CONFIG_NRF91_PROGMEM
  ret = nrf91_progmem_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MTD progmem: %d\n", ret);
    }
#endif /* CONFIG_MTD */

  UNUSED(ret);
  return OK;
}
