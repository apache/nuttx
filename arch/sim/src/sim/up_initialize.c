/****************************************************************************
 * arch/sim/src/sim/up_initialize.c
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

#include <nuttx/arch.h>
#include <nuttx/audio/audio.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/loop.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/net/loopback.h>
#include <nuttx/net/tun.h>
#include <nuttx/net/telnet.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/syslog/syslog_console.h>
#include <nuttx/serial/pty.h>
#include <nuttx/crypto/crypto.h>
#include <nuttx/power/pm.h>

#include "up_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_init_smartfs
 *
 * Description:
 *   Initialize a simulated SPI FLASH block device m25p MTD driver and bind
 *   it to a SMART Flash block device.
 *
 ****************************************************************************/

#if defined(CONFIG_FS_SMARTFS) && (defined(CONFIG_SIM_SPIFLASH) || defined(CONFIG_SIM_QSPIFLASH))
static void up_init_smartfs(void)
{
  FAR struct mtd_dev_s *mtd;
  int minor = 0;
#if defined(CONFIG_MTD_M25P) || defined(CONFIG_MTD_W25) || defined(CONFIG_MTD_SST26)
  FAR struct spi_dev_s *spi;
#endif
#ifdef CONFIG_MTD_N25QXXX
  FAR struct qspi_dev_s *qspi;
#endif

#ifdef CONFIG_SIM_SPIFLASH
#ifdef CONFIG_MTD_M25P
  /* Initialize a simulated SPI FLASH block device m25p MTD driver */

  spi = up_spiflashinitialize("m25p");
  if (spi != NULL)
    {
      mtd = m25p_initialize(spi);

      /* Now initialize a SMART Flash block device and bind it to the MTD
       * device
       */

      if (mtd != NULL)
        {
          smart_initialize(minor++, mtd, "_m25p");
        }
    }
#endif

#ifdef CONFIG_MTD_SST26
  /* Initialize a simulated SPI FLASH block device sst26 MTD driver */

  spi = up_spiflashinitialize("sst26");
  if (spi != NULL)
    {
      mtd = sst26_initialize_spi(spi);

      /* Now initialize a SMART Flash block device and bind it to the MTD
       * device
       */

      if (mtd != NULL)
        {
          smart_initialize(minor++, mtd, "_sst26");
        }
    }
#endif

#ifdef CONFIG_MTD_W25
  /* Initialize a simulated SPI FLASH block device w25 MTD driver */

  spi = up_spiflashinitialize("w25");
  if (spi != NULL)
    {
      mtd = w25_initialize(spi);

      /* Now initialize a SMART Flash block device and bind it to the MTD
       * device
       */

      if (mtd != NULL)
        {
          smart_initialize(minor++, mtd, "_w25");
        }
    }
#endif
#endif      /* CONFIG_SIM_SPIFLASH */

#if defined(CONFIG_MTD_N25QXXX) && defined(CONFIG_SIM_QSPIFLASH)
  /* Initialize a simulated SPI FLASH block device n25qxxx MTD driver */

  qspi = up_qspiflashinitialize();
  if (qspi != NULL)
    {
      mtd = n25qxxx_initialize(qspi, 0);

      /* Now initialize a SMART Flash block device and bind it to the MTD
       * device
       */

      if (mtd != NULL)
        {
          smart_initialize(minor++, mtd, "_n25q");
        }
    }
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS
 *   initialization after the basic OS services have been
 *   initialized.  The architecture specific details of
 *   initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the
 *   clock, and registering device drivers are some of the
 *   things that are different for each processor and hardware
 *   platform.
 *
 *   up_initialize is called after the OS initialized but
 *   before the init process has been started and before the
 *   libraries have been initialized.  OS services and driver
 *   services are available.
 *
 ****************************************************************************/

void up_initialize(void)
{
#ifdef CONFIG_PM
  /* Initialize the power management subsystem.  This MCU-specific function
   * must be called *very* early in the initialization sequence *before* any
   * other device drivers are initialized (since they may attempt to register
   * with the power management subsystem).
   */

  pm_initialize();
#endif

  /* Register devices */

#if defined(CONFIG_DEV_NULL)
  devnull_register();       /* Standard /dev/null */
#endif

#if defined(CONFIG_DEV_RANDOM)
  devrandom_register(); /* Standard /dev/random */
#endif

#if defined(CONFIG_DEV_URANDOM)
  devurandom_register();   /* Standard /dev/urandom */
#endif

#if defined(CONFIG_DEV_ZERO)
  devzero_register();       /* Standard /dev/zero */
#endif

#if defined(CONFIG_DEV_LOOP)
  loop_register();          /* Standard /dev/loop */
#endif

#if defined(CONFIG_DRIVER_NOTE)
  note_register();          /* Non-standard /dev/note */
#endif

#ifdef CONFIG_RPMSG_UART
  rpmsg_serialinit();
#endif

  /* Register some tty-port to access tty-port on sim platform */

  up_uartinit();

#if defined(CONFIG_CONSOLE_SYSLOG)
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

#if defined(CONFIG_FS_FAT) && !defined(CONFIG_DISABLE_MOUNTPOINT)
  up_registerblockdevice(); /* Our FAT ramdisk at /dev/ram0 */
#endif

#ifdef CONFIG_SIM_NETDEV
  netdriver_init();         /* Our "real" network driver */
#endif

#ifdef CONFIG_SIM_NETUSRSOCK
  /* Register the usrsock native socket device */

  usrsock_init();
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

#if defined(CONFIG_FS_SMARTFS) && (defined(CONFIG_SIM_SPIFLASH) || defined(CONFIG_SIM_QSPIFLASH))
  up_init_smartfs();
#endif

#ifdef CONFIG_SIM_SOUND
  audio_register("pcm0p", sim_audio_initialize(true));
  audio_register("pcm0c", sim_audio_initialize(false));
#endif
}
