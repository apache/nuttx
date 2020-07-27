/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_bringup.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/mount.h>
#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#ifdef CONFIG_RNDIS
#include <nuttx/usb/rndis.h>
#endif

#ifdef CONFIG_USERLED_LOWER
#include <nuttx/leds/userled.h>
#endif

#include <arch/chip/pm.h>
#include "chip.h"

#include "cxd56_sysctl.h"
#include "cxd56_powermgr.h"
#include "cxd56_uart.h"
#include "cxd56_timerisr.h"
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

#ifdef CONFIG_CXD56_PM_PROCFS
#  include "cxd56_powermgr_procfs.h"
#endif

#ifdef CONFIG_TIMER
#  include "cxd56_timer.h"
#endif

#ifdef CONFIG_WDT
#  include "cxd56_wdt.h"
#endif

#ifdef CONFIG_CXD56_RTC
#  include <nuttx/timers/rtc.h>
#  include "cxd56_rtc.h"
#endif

#ifdef CONFIG_CXD56_CPUFIFO
#  include "cxd56_cpufifo.h"
#endif

#ifdef CONFIG_CXD56_ICC
#  include "cxd56_icc.h"
#endif

#ifdef CONFIG_CXD56_FARAPI
#  include "cxd56_farapi.h"
#endif

#if defined(CONFIG_USBDEV) && defined(CONFIG_FS_PROCFS_REGISTER)
#  include "cxd56_usbdev.h"
#endif

#ifdef CONFIG_PWM
#  include <arch/board/cxd56_pwm.h>
#endif

#ifdef CONFIG_CXD56_ADC
#  include <arch/chip/adc.h>
#endif

#ifdef CONFIG_CXD56_SCU
#  include <arch/chip/scu.h>
#endif

#ifdef CONFIG_CXD56_GNSS
#  include "cxd56_gnss.h"
#endif

#ifdef CONFIG_CXD56_GEOFENCE
#  include "cxd56_geofence.h"
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#include "spresense.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
 #ifdef CONFIG_NSH_PROC_MOUNTPOINT
 #define CXD56_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
 #else
 #define CXD56_PROCFS_MOUNTPOINT "/proc"
 #endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_CXD56_CPUFIFO
static int nsh_cpucom_initialize(void)
{
  int ret = OK;

  cxd56_cfinitialize();

#ifdef CONFIG_CXD56_ICC
  cxd56_iccinitialize();
#endif
#ifdef CONFIG_CXD56_FARAPI
  cxd56_farapiinitialize();
#endif

  cxd56_sysctlinitialize();

  return ret;
}
#else
#  define nsh_cpucom_initialize() (OK)
#endif

#ifdef CONFIG_TIMER
static void timer_initialize(void)
{
  int i;
  char devname[16];

  for (i = 0; i < CXD56_TIMER_NUM; i++)
    {
      snprintf(devname, sizeof(devname), "/dev/timer%d", i);
      cxd56_timer_initialize(devname, i);
    }

  return;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int cxd56_bringup(void)
{
  struct pm_cpu_wakelock_s wlock;
  int ret;
#ifdef CONFIG_VIDEO_ISX012
  FAR const struct video_devops_s *devops;
#endif

  ret = nsh_cpucom_initialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize cpucom.\n");
    }

  ret = cxd56_pm_initialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize powermgr.\n");
    }

#ifdef CONFIG_CXD56_PM_PROCFS
  ret = cxd56_pm_initialize_procfs();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize powermgr.\n");
    }
#endif

  wlock.info = PM_CPUWAKELOCK_TAG('C', 'A', 0);
  wlock.count = 0;
  up_pm_acquire_wakelock(&wlock);

#ifdef CONFIG_RTC_DRIVER
  rtc_initialize(0, cxd56_rtc_lowerhalf());
#endif

#ifdef CONFIG_TIMER
  timer_initialize();
#endif

#ifdef CONFIG_CXD56_WDT
  ret = cxd56_wdt_initialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize WDT.\n");
    }
#endif

  cxd56_uart_initialize();
  cxd56_timerisr_initialize();

#ifdef CONFIG_CXD56_CPUFIFO
  ret = cxd56_pm_bootup();
  if (ret < 0)
    {
      _err("ERROR: Failed to powermgr bootup.\n");
    }
#endif

#ifndef CONFIG_CXD56_SUBCORE
  /* Initialize CPU clock to max frequency */

  board_clock_initialize();

  /* Setup the power of external device */

  board_power_setup(0);
#endif

#ifdef CONFIG_CXD56_SCU
  scu_initialize();
#endif

#ifdef CONFIG_CXD56_I2C_DRIVER
  #ifdef CONFIG_CXD56_I2C0
  ret = board_i2cdev_initialize(0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize I2C0.\n");
    }
  #endif

  #ifdef CONFIG_CXD56_I2C1
  ret = board_i2cdev_initialize(1);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize I2C1.\n");
    }
  #endif

  #ifdef CONFIG_CXD56_I2C2
  ret = board_i2cdev_initialize(2);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize I2C2.\n");
    }
  #endif
#endif

#ifdef CONFIG_SYSTEM_SPITOOL
#  ifdef CONFIG_CXD56_SPI3
  ret = board_spidev_initialize(3);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize SPI3.\n");
    }
#  endif

#  ifdef CONFIG_CXD56_SPI4
  ret = board_spidev_initialize(4);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize SPI4.\n");
    }
#  endif

#  ifdef CONFIG_CXD56_SPI5
  ret = board_spidev_initialize(5);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize SPI5.\n");
    }
#  endif
#endif

#ifdef CONFIG_FS_PROCFS

#if defined(CONFIG_USBDEV) && defined(CONFIG_FS_PROCFS_REGISTER)
  /* Register usbdev procfs */

  ret = cxd56_usbdev_procfs_register();
  if (ret < 0)
    {
      _err("ERROR: Failed to register usbdev.\n");
    }
#endif

  ret = mount(NULL, CXD56_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      _err("ERROR: Failed to mount the procfs: %d\n", errno);
    }
#endif

#ifdef CONFIG_PWM
  ret = board_pwm_setup();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize pwm. \n");
    }
#endif

#ifdef CONFIG_CXD56_ADC
  ret = cxd56_adcinitialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize adc. \n");
    }
#endif

#ifdef CONFIG_USERLED_LOWER
  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize led. \n");
    }
#endif

#ifdef CONFIG_CXD56_SFC
  ret = board_flash_initialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize SPI-Flash. %d\n", errno);
    }
#endif

#ifdef CONFIG_AUDIO_CXD56
  ret = board_audio_initialize_driver(1);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize audio. %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_ISX012
  ret = board_isx012_initialize(IMAGER_I2C);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize ISX012 board. %d\n", errno);
    }

  devops  = isx012_initialize();
  if (devops == NULL)
    {
      _err("ERROR: Failed to populate ISX012 devops. %d\n", errno);
      ret = ERROR;
    }
#endif /* CONFIG_VIDEO_ISX012 */

#if defined(CONFIG_CXD56_SDIO)
  /* In order to prevent Hi-Z from being input to the SD Card controller,
   * Initialize SDIO pins to GPIO low output with internal pull-down.
   */

  CXD56_PIN_CONFIGS(PINCONFS_SDIOA_GPIO);
  cxd56_gpio_write(PIN_SDIO_CLK, false);
  cxd56_gpio_write(PIN_SDIO_CMD, false);
  cxd56_gpio_write(PIN_SDIO_DATA0, false);
  cxd56_gpio_write(PIN_SDIO_DATA1, false);
  cxd56_gpio_write(PIN_SDIO_DATA2, false);
  cxd56_gpio_write(PIN_SDIO_DATA3, false);

  ret = board_sdcard_initialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize sdhci. \n");
    }
#endif

#ifdef CONFIG_CXD56_SPISD
  /* Mount the SPI-based MMC/SD block driver */

  ret = board_spisd_initialize(0, CONFIG_CXD56_SPISD_SPI_CH);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize SPI device to MMC/SD: %d\n",
           ret);
    }
#endif

#ifdef CONFIG_CPUFREQ_RELEASE_LOCK
  /* Enable dynamic clock control and CPU clock down for power saving */

  board_clock_enable();
#endif

  up_pm_release_wakelock(&wlock);

#if defined(CONFIG_RNDIS)
  uint8_t mac[6];
  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

#ifdef CONFIG_WL_GS2200M
  ret = board_gs2200m_initialize("/dev/gs2200m", 5);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize GS2200M. \n");
    }
#endif

#ifdef CONFIG_CXD56_GNSS
  ret = cxd56_gnssinitialize("/dev/gps");
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize gnss. \n");
    }
#endif

#ifdef CONFIG_CXD56_GEOFENCE
  ret = cxd56_geofenceinitialize("/dev/geofence");
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize geofence. \n");
    }
#endif

#ifdef CONFIG_SENSORS
  ret = board_sensors_initialize();
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize sensors.\n");
    }
#endif

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      _err("ERROR: Failed to initialize Frame Buffer Driver.\n");
    }
#endif

  return 0;
}
