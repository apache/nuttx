/****************************************************************************
 * arch/sim/src/sim/sim_initialize.c
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

#include <nuttx/arch.h>
#include <nuttx/audio/audio.h>
#include <nuttx/kthread.h>
#include <nuttx/motor/foc/foc_dummy.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/power/pm.h>
#include <nuttx/spi/spi_flash.h>
#include <nuttx/spi/qspi_flash.h>

#include <stdlib.h>

#include "sim_internal.h"
#include "sim_hostusrsock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON
static void sim_init_cmdline(void)
{
  char cmdline[ARG_MAX] = "";
  int i;

  for (i = 1; i < g_argc; i++)
    {
      strlcat(cmdline, g_argv[i], sizeof(cmdline));
      strlcat(cmdline, " ", sizeof(cmdline));
    }

  setenv("CMDLINE", cmdline, true);
}
#endif

/****************************************************************************
 * Name: sim_init_smartfs
 *
 * Description:
 *   Initialize a simulated SPI FLASH block device m25p MTD driver and bind
 *   it to a SMART Flash block device.
 *
 ****************************************************************************/

#if defined(CONFIG_FS_SMARTFS) && defined(CONFIG_MTD_SMART) && \
    (defined(CONFIG_SPI_FLASH) || defined(CONFIG_QSPI_FLASH))
static void sim_init_smartfs(void)
{
#if defined(CONFIG_MTD_M25P) || defined(CONFIG_MTD_W25) || defined(CONFIG_MTD_SST26)
  struct mtd_dev_s *mtd;
  struct spi_dev_s *spi;
  int minor = 0;
#endif
#ifdef CONFIG_MTD_N25QXXX
  struct qspi_dev_s *qspi;
#endif

#ifdef CONFIG_SPI_FLASH
#ifdef CONFIG_MTD_M25P
  /* Initialize a simulated SPI FLASH block device m25p MTD driver */

  spi = spi_flash_initialize("m25p");
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

  spi = spi_flash_initialize("sst26");
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

  spi = spi_flash_initialize("w25");
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
#endif /* CONFIG_SPI_FLASH */

#if defined(CONFIG_MTD_N25QXXX) && defined(CONFIG_QSPI_FLASH)
  /* Initialize a simulated SPI FLASH block device n25qxxx MTD driver */

  qspi = qspi_flash_initialize();
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

static int sim_loop_task(int argc, char **argv)
{
  while (1)
    {
      irqstate_t flags = up_irq_save();

      sched_lock();

#if defined(CONFIG_SIM_TOUCHSCREEN) || defined(CONFIG_SIM_AJOYSTICK) || \
    defined(CONFIG_SIM_BUTTONS)
      /* Drive the X11 event loop */

      sim_x11events();
#endif

#if defined(CONFIG_SIM_LCDDRIVER) || defined(CONFIG_SIM_FRAMEBUFFER)
      sim_x11loop();
#endif

#ifdef CONFIG_SIM_NETDEV
      /* Run the network if enabled */

      sim_netdriver_loop();
#endif

#ifdef CONFIG_SIM_NETUSRSOCK
      host_usrsock_loop();
#endif

#ifdef CONFIG_RPTUN
      sim_rptun_loop();
#endif

#ifdef CONFIG_SIM_HCISOCKET
      sim_bthcisock_loop();
#endif

#ifdef CONFIG_SIM_SOUND
      sim_audio_loop();
#endif

#ifdef CONFIG_SIM_VIDEO
      sim_video_loop();
#endif

#ifdef CONFIG_SIM_USB_DEV
      sim_usbdev_loop();
#endif

#ifdef CONFIG_MOTOR_FOC_DUMMY
      /* Update simulated FOC device */

      foc_dummy_update();
#endif

      sched_unlock();
      up_irq_restore(flags);

      /* Sleep minimal time, let the idle run */

      usleep(CONFIG_SIM_LOOPTASK_INTERVAL);
    }

  return 0;
}

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

#ifndef CONFIG_DISABLE_ENVIRON
  sim_init_cmdline();
#endif

  /* Register some tty-port to access tty-port on sim platform */

  sim_uartinit();

#if defined(CONFIG_FS_FAT) && !defined(CONFIG_DISABLE_MOUNTPOINT)
  sim_registerblockdevice(); /* Our FAT ramdisk at /dev/ram0 */
#endif

#ifdef CONFIG_SIM_NETDEV
  sim_netdriver_init();         /* Our "real" network driver */
#endif

#if defined(CONFIG_FS_SMARTFS) && defined(CONFIG_MTD_SMART) && \
    (defined(CONFIG_SPI_FLASH) || defined(CONFIG_QSPI_FLASH))
  sim_init_smartfs();
#endif

#ifdef CONFIG_SIM_SOUND
  /* Register audio normal device */

  audio_register("pcm0p", sim_audio_initialize(true, false));
  audio_register("pcm0c", sim_audio_initialize(false, false));

  /* Register audio compress device */

  audio_register("pcm1p", sim_audio_initialize(true, true));
  audio_register("pcm1c", sim_audio_initialize(false, true));
#endif

#ifdef CONFIG_SIM_USB_DEV
  sim_usbdev_initialize();
#endif

  kthread_create("loop_task", SCHED_PRIORITY_MAX,
                 CONFIG_DEFAULT_TASK_STACKSIZE,
                 sim_loop_task, NULL);
}
