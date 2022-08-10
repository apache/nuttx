/****************************************************************************
 * drivers/drivers_initialize.c
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

#include <nuttx/clk/clk_provider.h>
#include <nuttx/crypto/crypto.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/loop.h>
#include <nuttx/input/uinput.h>
#include <nuttx/net/loopback.h>
#include <nuttx/net/tun.h>
#include <nuttx/net/telnet.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/power/pm.h>
#include <nuttx/power/regulator.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/serial/pty.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/syslog/syslog_console.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drivers_initialize
 *
 * Description:
 *   drivers_initialize will be called once during OS initialization after
 *   the basic OS services have been initialized.
 *
 *   drivers_initialize is called after the OS initialized but before the
 *   user initialization logic has been started and before the libraries
 *   have been initialized.  OS services and driver services are available.
 *
 ****************************************************************************/

void drivers_initialize(void)
{
  /* Register devices */

  syslog_initialize();

#if defined(CONFIG_DEV_NULL)
  devnull_register();   /* Standard /dev/null */
#endif

#if defined(CONFIG_DEV_RANDOM)
  devrandom_register(); /* Standard /dev/random */
#endif

#if defined(CONFIG_DEV_URANDOM)
  devurandom_register();   /* Standard /dev/urandom */
#endif

#if defined(CONFIG_DEV_ZERO)
  devzero_register();   /* Standard /dev/zero */
#endif

#if defined(CONFIG_DEV_LOOP)
  loop_register();      /* Standard /dev/loop */
#endif

#if defined(CONFIG_DRIVER_NOTE)
  note_register();      /* Non-standard /dev/note */
#endif

#if defined(CONFIG_CLK_RPMSG)
  clk_rpmsg_server_initialize();
#endif

#if defined(CONFIG_REGULATOR_RPMSG)
  regulator_rpmsg_server_init();
#endif

  /* Initialize the serial device driver */

#ifdef CONFIG_RPMSG_UART
  rpmsg_serialinit();
#endif

  /* Initialize the console device driver (if it is other than the standard
   * serial driver).
   */

#if defined(CONFIG_LWL_CONSOLE)
  lwlconsole_init();
#elif defined(CONFIG_CONSOLE_SYSLOG)
  syslog_console_init();
#endif

#ifdef CONFIG_PSEUDOTERM_SUSV1
  /* Register the master pseudo-terminal multiplexor device */

  ptmx_register();
#endif

#if defined(CONFIG_CRYPTO)
  /* Initialize the HW crypto and /dev/crypto */

  up_cryptoinitialize();
#endif

#ifdef CONFIG_CRYPTO_CRYPTODEV
  devcrypto_register();
#endif

#ifdef CONFIG_UINPUT_TOUCH
  uinput_touch_initialize();
#endif

#ifdef CONFIG_UINPUT_BUTTONS
  uinput_button_initialize();
#endif

#ifdef CONFIG_UINPUT_KEYBOARD
  uinput_keyboard_initialize();
#endif

#ifdef CONFIG_NET_LOOPBACK
  /* Initialize the local loopback device */

  localhost_initialize();
#endif

#ifdef CONFIG_NET_TUN
  /* Initialize the TUN device */

  tun_initialize();
#endif

#ifdef CONFIG_NETDEV_TELNET
  /* Initialize the Telnet session factory */

  telnet_initialize();
#endif

#ifdef CONFIG_USENSOR
  usensor_initialize();
#endif

#ifdef CONFIG_SENSORS_RPMSG
  sensor_rpmsg_initialize();
#endif
}
