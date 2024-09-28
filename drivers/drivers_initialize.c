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
#include <nuttx/drivers/rpmsgdev.h>
#include <nuttx/drivers/rpmsgblk.h>
#include <nuttx/fs/loop.h>
#include <nuttx/fs/smart.h>
#include <nuttx/fs/loopmtd.h>
#include <nuttx/input/uinput.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/net/loopback.h>
#include <nuttx/net/tun.h>
#include <nuttx/net/telnet.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/pci/pci.h>
#include <nuttx/power/pm.h>
#include <nuttx/power/regulator.h>
#include <nuttx/reset/reset-controller.h>
#include <nuttx/segger/rtt.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/serial/pty.h>
#include <nuttx/serial/uart_hostfs.h>
#include <nuttx/serial/uart_ram.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/syslog/syslog_console.h>
#include <nuttx/thermal.h>
#include <nuttx/trace.h>
#include <nuttx/usrsock/usrsock_rpmsg.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/drivers/optee.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if only one console device is selected.
 * If you get this errro, search your .config file for CONSOLE_XXX_CONSOLE
 * options and remove what is not needed.
 */

#if (defined(CONFIG_LWL_CONSOLE) + defined(CONFIG_SERIAL_CONSOLE) + \
     defined(CONFIG_CDCACM_CONSOLE) + defined(CONFIG_PL2303_CONSOLE) + \
     defined(CONFIG_SERIAL_RTT_CONSOLE) + defined(CONFIG_RPMSG_UART_CONSOLE)) > 1
#  error More than one console driver selected. Check your configuration !
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drivers_early_initialize
 *
 * Description:
 *   drivers_early_initialize will be called once before OS initialization
 *   when no system resource is ready to use.
 *
 *   drivers_early_initialize serves the purpose of bringing up drivers as
 *   early as possible, so they can be used even during OS initialization.
 *   It must not rely on any system resources, such as heap memory.
 *
 ****************************************************************************/

void drivers_early_initialize(void)
{
#ifdef CONFIG_DRIVERS_NOTE
  note_early_initialize();
#endif
}

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
  drivers_trace_begin();

  /* Register devices */

  syslog_initialize();

#ifdef CONFIG_SERIAL_RTT
  serial_rtt_initialize();
#endif

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

#ifdef CONFIG_DEV_MEM
  devmem_register();
#endif

#if defined(CONFIG_DEV_LOOP)
  loop_register();      /* Standard /dev/loop */
#endif

#if defined(CONFIG_DEV_ASCII)
  devascii_register();  /* Non-standard /dev/ascii */
#endif

#if defined(CONFIG_DRIVERS_NOTE)
  note_initialize();    /* Non-standard /dev/note */
#endif

#if defined(CONFIG_CLK_RPMSG)
  clk_rpmsg_server_initialize();
#endif

#if defined(CONFIG_REGULATOR_RPMSG)
  regulator_rpmsg_server_init();
#endif

#if defined(CONFIG_RESET_RPMSG)
  reset_rpmsg_server_init();
#endif

  /* Initialize the serial device driver */

#ifdef CONFIG_RPMSG_UART
  rpmsg_serialinit();
#endif

#ifdef CONFIG_RAM_UART
  ram_serialinit();
#endif

  /* Initialize the console device driver (if it is other than the standard
   * serial driver).
   */

#if defined(CONFIG_LWL_CONSOLE)
  lwlconsole_init();
#elif defined(CONFIG_CONSOLE_SYSLOG)
  syslog_console_init();
#endif

#ifdef CONFIG_UART_HOSTFS
  uart_hostfs_init();
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

#ifdef CONFIG_DEV_RPMSG_SERVER
  rpmsgdev_server_init();
#endif

#ifdef CONFIG_BLK_RPMSG_SERVER
  rpmsgblk_server_init();
#endif

#ifdef CONFIG_RPMSGMTD_SERVER
  rpmsgmtd_server_init();
#endif

#ifdef CONFIG_NET_USRSOCK_RPMSG_SERVER
  /* Initialize the user socket rpmsg server */

  usrsock_rpmsg_server_initialize();
#endif

#ifdef CONFIG_SMART_DEV_LOOP
  smart_loop_register_driver();
#endif

#ifdef CONFIG_MTD_LOOP
  mtd_loop_register();
#endif

#if defined(CONFIG_PCI) && !defined(CONFIG_PCI_LATE_DRIVERS_REGISTER)
  pci_register_drivers();
#endif

#ifdef CONFIG_DRIVERS_VIRTIO
  virtio_register_drivers();
#endif

#ifndef CONFIG_DEV_OPTEE_NONE
  optee_register();
#endif

#ifdef CONFIG_THERMAL
  thermal_init();
#endif

  drivers_trace_end();
}
