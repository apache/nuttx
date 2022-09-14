/****************************************************************************
 * boards/sim/sim/sim/src/sim_bringup.c
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
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/drivers/rpmsgdev.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/rc/lirc_dev.h>
#include <nuttx/rc/dummy.h>
#include <nuttx/sensors/fakesensor.h>
#include <nuttx/sensors/mpu60x0.h>
#include <nuttx/sensors/wtgahrs2.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/video/fb.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/wireless/pktradio.h>
#include <nuttx/wireless/bluetooth/bt_null.h>
#include <nuttx/wireless/bluetooth/bt_uart_shim.h>
#include <nuttx/wireless/ieee802154/ieee802154_loopback.h>

#ifdef CONFIG_LCD_DEV
#include <nuttx/lcd/lcd_dev.h>
#endif

#if defined(CONFIG_INPUT_BUTTONS_LOWER) && defined(CONFIG_SIM_BUTTONS)
#include <nuttx/input/buttons.h>
#endif

#include "up_internal.h"
#include "sim.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
#ifdef CONFIG_SIM_RPTUN_MASTER
  uart_rpmsg_init("proxy", "proxy", 4096, false);
#else
  uart_rpmsg_init("server", "proxy", 4096, true);
#endif
}
#endif

/****************************************************************************
 * Name: sim_bringup
 *
 * Description:
 *   Bring up simulated board features
 *
 ****************************************************************************/

int sim_bringup(void)
{
#ifdef CONFIG_ONESHOT
  struct oneshot_lowerhalf_s *oneshot;
#endif
#ifdef CONFIG_RAMMTD
  uint8_t *ramstart;
#endif
#ifdef CONFIG_SIM_I2CBUS
  struct i2c_master_s *i2cbus;
#endif
#ifdef CONFIG_MPU60X0_I2C
  struct mpu_config_s *mpu_config;
#endif
#ifdef CONFIG_SIM_SPI
  struct spi_dev_s *spidev;
#endif
  int ret = OK;

#ifdef CONFIG_FS_BINFS
  /* Mount the binfs file system */

  ret = nx_mount(NULL, "/bin", "binfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount binfs at /bin: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, SIM_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             SIM_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#ifdef CONFIG_LIBC_ZONEINFO_ROMFS
  /* Mount the TZ database */

  sim_zoneinfo(3);
#endif

#ifdef CONFIG_EXAMPLES_GPIO
  /* Initialize simulated GPIO drivers */

  sim_gpio_initialize();
#endif

#ifdef CONFIG_RAMMTD
  /* Create a RAM MTD device if configured */

  ramstart = (uint8_t *)kmm_malloc(128 * 1024);
  if (ramstart == NULL)
    {
      syslog(LOG_ERR, "ERROR: Allocation for RAM MTD failed\n");
    }
  else
    {
      /* Initialized the RAM MTD */

      struct mtd_dev_s *mtd = rammtd_initialize(ramstart, 128 * 1024);
      if (mtd == NULL)
        {
          syslog(LOG_ERR, "ERROR: rammtd_initialize failed\n");
          kmm_free(ramstart);
        }
      else
        {
          /* Erase the RAM MTD */

          ret = mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: IOCTL MTDIOC_BULKERASE failed\n");
            }

          /* Register the MTD driver so that it can be accessed from the
           * VFS.
           */

          ret = register_mtddriver("/dev/rammtd", mtd, 0755, NULL);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n",
                     ret);
            }

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
          /* Initialize a SMART Flash block device and bind it to the MTD
           * device.
           */

          smart_initialize(0, mtd, NULL);

#elif defined(CONFIG_FS_SPIFFS)
          /* Mount the SPIFFS file system */

          ret = nx_mount("/dev/rammtd", "/mnt/spiffs", "spiffs", 0, NULL);
          if (ret < 0)
            {
              syslog(LOG_ERR,
                     "ERROR: Failed to mount SPIFFS at /mnt/spiffs: %d\n",
                     ret);
            }

#elif defined(CONFIG_FS_LITTLEFS)
          /* Mount the LittleFS file system */

          ret = nx_mount("/dev/rammtd", "/mnt/lfs", "littlefs", 0,
                         "forceformat");
          if (ret < 0)
            {
              syslog(LOG_ERR,
                     "ERROR: Failed to mount LittleFS at /mnt/lfs: %d\n",
                     ret);
            }

#elif defined(CONFIG_FS_NXFFS)
          /* Initialize to provide NXFFS on the MTD interface */

          ret = nxffs_initialize(mtd);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: NXFFS initialization failed: %d\n",
                     ret);
            }
#endif
        }
    }
#endif

#ifdef CONFIG_ONESHOT
  /* Get an instance of the simulated oneshot timer */

  oneshot = oneshot_initialize(0, 0);
  if (oneshot == NULL)
    {
      syslog(LOG_ERR, "ERROR: oneshot_initialize failed\n");
    }
  else
    {
#ifdef CONFIG_CPULOAD_ONESHOT
      /* Configure the oneshot timer to support CPU load measurement */

      nxsched_oneshot_extclk(oneshot);

#else
      /* Initialize the simulated oneshot driver */

      ret = oneshot_register("/dev/oneshot", oneshot);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to register oneshot at /dev/oneshot: %d\n",
                 ret);
        }
#endif
    }
#endif

#ifdef CONFIG_INPUT_AJOYSTICK
  /* Initialize the simulated analog joystick input device */

  sim_ajoy_initialize();
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the simulated framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LCD

  ret = board_lcd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_lcd_initialize() failed: %d\n", ret);
    }

#  ifdef CONFIG_LCD_DEV

  ret = lcddev_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lcddev_register() failed: %d\n", ret);
    }

#  endif

#endif

#ifdef CONFIG_SIM_TOUCHSCREEN
  /* Initialize the touchscreen */

  ret = sim_tsc_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sim_tsc_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SIM_KEYBOARD
  /* Initialize the keyboard */

  ret = sim_kbd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sim_kbd_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_IEEE802154_LOOPBACK
  /* Initialize and register the IEEE802.15.4 MAC network loop device */

  ret = ieee8021514_loopback();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ieee8021514_loopback() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_PKTRADIO_LOOPBACK
  /* Initialize and register the IEEE802.15.4 MAC network loop device */

  ret = pktradio_loopback();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pktradio_loopback() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_BLUETOOTH_NULL
  /* Register the NULL Bluetooth network device */

  ret = btnull_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btnull_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SIM_HCISOCKET
  /* Register the Host Bluetooth network device via HCI socket */

  ret = bthcisock_register(CONFIG_SIM_HCISOCKET_DEVID);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bthcisock_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SIM_I2CBUS
  /* Initialize the i2c master bus device */

  i2cbus = sim_i2cbus_initialize(CONFIG_SIM_I2CBUS_ID);
  if (i2cbus == NULL)
    {
      syslog(LOG_ERR, "ERROR: sim_i2cbus_initialize failed.\n");
    }
#if defined(CONFIG_SYSTEM_I2CTOOL) || defined(CONFIG_MPU60X0_I2C)
  else
    {
      ret = i2c_register(i2cbus, 0);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 0, ret);
          sim_i2cbus_uninitialize(i2cbus);
        }

#ifdef CONFIG_MPU60X0_I2C
      mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
      if (mpu_config == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to allocate mpu60x0 driver\n");
        }
      else
        {
          mpu_config->i2c = i2cbus;
          mpu_config->addr = 0x68;
          mpu60x0_register("/dev/imu0", mpu_config);
        }
#endif
    }
#endif
#endif

#ifdef CONFIG_SIM_SPI
  spidev = sim_spi_initialize(CONFIG_SIM_SPIDEV_NAME);
  if (spidev == NULL)
    {
      syslog(LOG_ERR, "ERROR: sim_spi_initialize failed.\n");
    }
#ifdef CONFIG_SYSTEM_SPITOOL
  else
    {
      ret = spi_register(spidev, 0);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register SPI%d driver: %d\n",
                 0, ret);
          sim_spi_uninitialize(spidev);
        }
    }
#endif /* CONFIG_SYSTEM_SPITOOL */
#endif /* CONFIG_SIM_SPI */

#if defined(CONFIG_INPUT_BUTTONS_LOWER) && defined(CONFIG_SIM_BUTTONS)
  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_MOTOR_FOC_DUMMY
  /* Setup FOC device */

  ret = sim_foc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sim_foc_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RPTUN
#ifdef CONFIG_SIM_RPTUN_MASTER
  up_rptun_init("server-proxy", "proxy", true);
#else
  up_rptun_init("server-proxy", "server", false);
#endif

#ifdef CONFIG_DEV_RPMSG
  rpmsgdev_register("server", "/dev/console", "/dev/server-console");
  rpmsgdev_register("server", "/dev/null", "/dev/server-null");
  rpmsgdev_register("server", "/dev/ttyUSB0", "/dev/ttyUSB0");
#endif

#ifdef CONFIG_RPMSGMTD
  rpmsgmtd_register("server", "/dev/rammtd", NULL);
#endif
#endif

#ifdef CONFIG_SIM_WTGAHRS2_UARTN
#if CONFIG_SIM_WTGAHRS2_UARTN == 0
  wtgahrs2_initialize(CONFIG_SIM_UART0_NAME, 0);
#elif CONFIG_SIM_WTGAHRS2_UARTN == 1
  wtgahrs2_initialize(CONFIG_SIM_UART1_NAME, 1);
#elif CONFIG_SIM_WTGAHRS2_UARTN == 2
  wtgahrs2_initialize(CONFIG_SIM_UART2_NAME, 2);
#elif CONFIG_SIM_WTGAHRS2_UARTN == 3
  wtgahrs2_initialize(CONFIG_SIM_UART3_NAME, 3);
#endif
#endif

#ifdef CONFIG_RC_DUMMY
  rc_dummy_initialize(0);
#endif

  return ret;
}
