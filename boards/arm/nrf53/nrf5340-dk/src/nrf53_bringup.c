/****************************************************************************
 * boards/arm/nrf53/nrf5340-dk/src/nrf53_bringup.c
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
#ifdef CONFIG_RPMSG_UART
#  include <nuttx/serial/uart_rpmsg.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#ifdef CONFIG_NRF53_SOFTDEVICE_CONTROLLER
#  include "nrf53_sdc.h"
#endif

#ifdef CONFIG_RPTUN
#  include <nuttx/wireless/bluetooth/bt_rpmsghci.h>
#  ifdef CONFIG_UART_BTH4
#    include <nuttx/serial/uart_bth4.h>
#  endif
#  ifdef CONFIG_NET_BLUETOOTH
#    include <nuttx/wireless/bluetooth/bt_driver.h>
#  endif
#  include "nrf53_rptun.h"
#endif

#ifdef CONFIG_NRF53_PROGMEM
#  include "nrf53_progmem.h"
#endif

#ifdef CONFIG_TIMER
#  include "nrf53_timer.h"
#endif

#include "nrf5340-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF53_TIMER (0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_NRF53_APPCORE

/****************************************************************************
 * Name: nrf53_appcore_bleinit
 ****************************************************************************/

static int nrf53_appcore_bleinit(void)
{
  int ret = OK;

#ifdef CONFIG_BLUETOOTH_RPMSG
  struct bt_driver_s *bt_dev = NULL;

  bt_dev = rpmsghci_register("appcore", "bthci");
  if (bt_dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: rpmsghci_register() failed: %d\n", -errno);
      return -ENOMEM;
    }

#  ifdef CONFIG_UART_BTH4
  /* Register UART BT H4 device */

  ret = uart_bth4_register("/dev/ttyHCI", bt_dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "bt_bth4_register error: %d\n", ret);
    }
#  elif defined(CONFIG_NET_BLUETOOTH)
  /* Register network device */

  ret = bt_netdev_register(bt_dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "bt_netdev_register error: %d\n", ret);
    }
#  else
#    error
#  endif
#endif

  return ret;
}

#else

/****************************************************************************
 * Name: nrf53_netcore_bleinit
 ****************************************************************************/

static int nrf53_netcore_bleinit(void)
{
  int ret = OK;

#ifdef CONFIG_NRF53_SOFTDEVICE_CONTROLLER
#  ifdef CONFIG_BLUETOOTH_RPMSG_SERVER
  ret = nrf53_rpmsghci_server_initialize("bthci");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: nrf53_rpmsghci_server_initialize() failed: %d\n",
             ret);
    }
#  else
  ret = nrf53_sdc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf53_sdc_initialize() failed: %d\n", ret);
    }
#  endif
#endif

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
#ifdef CONFIG_NRF53_APPCORE
  uart_rpmsg_init("appcore", "proxy", 4096, false);
#else
  uart_rpmsg_init("netcore", "proxy", 4096, true);
#endif
}
#endif

/****************************************************************************
 * Name: nrf53_bringup
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

int nrf53_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, NRF53_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize(CONFIG_EXAMPLES_LEDS_DEVPATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RPTUN
#ifdef CONFIG_NRF53_APPCORE
  nrf53_rptun_init("nrf53-shmem", "appcore");
#else
  nrf53_rptun_init("nrf53-shmem", "netcore");
#endif
#endif

#if defined(CONFIG_TIMER) && defined(CONFIG_NRF53_TIMER)
  /* Configure TIMER driver */

  ret = nrf53_timer_driver_setup("/dev/timer0", NRF53_TIMER);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Configure PWM driver */

  ret = nrf53_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize PWM driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Configure ADC driver */

  ret = nrf53_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize ADC driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

#if defined(CONFIG_RNDIS) && !defined(CONFIG_RNDIS_COMPOSITE)
  uint8_t mac[6];
  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

#ifdef CONFIG_NRF53_QSPI
  /* Initialize the MX25 QSPU memory */

  ret = nrf53_mx25_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf53_mx25_initialize() failed: %d\n", ret);
    }
#endif

  /* Initialize BLE */

#ifdef CONFIG_NRF53_APPCORE
  ret = nrf53_appcore_bleinit();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf53_appcore_bleinit failed: %d\n", ret);
    }
#else
  ret = nrf53_netcore_bleinit();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf53_netcore_bleinit failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_NRF53_PROGMEM
  ret = nrf53_progmem_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MTD progmem: %d\n", ret);
    }
#endif /* CONFIG_MTD */

  UNUSED(ret);
  return OK;
}
