/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_sdcard.c
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

#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mmcsd.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>

#include "chip.h"
#include "arm_internal.h"

#include <arch/board/board.h>
#include <arch/chip/pin.h>
#include <arch/chip/pm.h>
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"
#include "cxd56_sdhci.h"
#include "hardware/cxd5602_topreg.h"

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
#  include "cxd56_gpioint.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TXS02612RTWR: SDIO port expander with voltage level translation */

#define SDCARD_TXS02612_SEL PIN_AP_CLK

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cxd56_sdhci_state_s
{
  struct sdio_dev_s *sdhci;   /* R/W device handle */
  bool initialized;           /* TRUE: SDHCI block driver is initialized */
#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  bool inserted;              /* TRUE: card is inserted */
#endif
  void (*cb)(bool);           /* Callback function pointer to application */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56_sdhci_state_s g_sdhci;
#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
static struct work_s g_sdcard_work;
#endif

static struct pm_cpu_freqlock_s g_hv_lock =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('S', 'D', 0),
                      PM_CPUFREQLOCK_FLAG_HV);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sdcard_enable
 *
 * Description:
 *   Enable SD Card on the board.
 *
 ****************************************************************************/

static void board_sdcard_enable(void *arg)
{
  struct stat stat_sdio;
  int ret = OK;

  /* Acquire frequency lock */

  up_pm_acquire_freqlock(&g_hv_lock);

  if (!g_sdhci.initialized)
    {
      /* Mount the SDHC-based MMC/SD block driver
       * This should be used with 3.3V
       * First, get an instance of the SDHC interface
       */

      finfo("Initializing SDHC slot 0\n");

      g_sdhci.sdhci = cxd56_sdhci_initialize(0);
      if (g_sdhci.sdhci == NULL)
        {
          _err("ERROR: Failed to initialize SDHC slot 0\n");
          goto release_frequency_lock;
        }

      /* If not initialize SD slot */

      if (nx_stat("/dev/mmcsd0", &stat_sdio, 1) != 0)
        {
          /* Now bind the SDHC interface to the MMC/SD driver */

          finfo("Bind SDHC to the MMC/SD driver, minor=0\n");

          ret = mmcsd_slotinitialize(0, g_sdhci.sdhci);
          if (ret != OK)
            {
              _err("ERROR: Failed to bind SDHC to the MMC/SD driver: %d\n",
                   ret);
              goto release_frequency_lock;
            }

          finfo("Successfully bound SDHC to the MMC/SD driver\n");
        }

      /* Handle the initial card state */

      cxd56_sdhci_mediachange(g_sdhci.sdhci);

#ifndef CONFIG_CXD56_SDCARD_AUTOMOUNT
      if (nx_stat("/dev/mmcsd0", &stat_sdio, 1) == 0)
        {
          if (S_ISBLK(stat_sdio.st_mode))
            {
              ret = nx_mount("/dev/mmcsd0", "/mnt/sd0", "vfat", 0, NULL);
              if (ret == 0)
                {
                  finfo(
                     "Successfully mount a SDCARD via the MMC/SD driver\n");
                }
              else
                {
                  _err("ERROR: Failed to mount the SDCARD. %d\n", ret);
                  cxd56_sdio_resetstatus(g_sdhci.sdhci);
                  goto release_frequency_lock;
                }
            }
        }

      /* Callback to application to notice card is inserted */

      if (g_sdhci.cb != NULL)
        {
          g_sdhci.cb(true);
        }
#else
      /* Let the automounter know about the insertion event */

      board_automount_event(0, board_sdcard_inserted(0));
#endif /* CONFIG_CXD56_SDCARD_AUTOMOUNT */

      g_sdhci.initialized = true;
    }

release_frequency_lock:

  /* Release frequency lock */

  up_pm_release_freqlock(&g_hv_lock);
}

/****************************************************************************
 * Name: board_sdcard_disable
 *
 * Description:
 *   Disable SD Card on the board.
 *
 ****************************************************************************/

static void board_sdcard_disable(void *arg)
{
  if (g_sdhci.initialized)
    {
#ifndef CONFIG_CXD56_SDCARD_AUTOMOUNT
      int ret;

      /* un-mount */

      ret = nx_umount2("/mnt/sd0", 0);
      if (ret < 0)
        {
          ferr("ERROR: Failed to unmount the SD Card: %d\n", ret);
        }

      /* Callback to application to notice card is ejected */

      if (g_sdhci.cb != NULL)
        {
          g_sdhci.cb(false);
        }
#endif /* CONFIG_CXD56_SDCARD_AUTOMOUNT */

      /* Report the new state to the SDIO driver */

      cxd56_sdhci_mediachange(g_sdhci.sdhci);

      cxd56_sdhci_finalize(0);

#ifdef CONFIG_CXD56_SDCARD_AUTOMOUNT
      /* Let the automounter know about the removal event */

      board_automount_event(0, board_sdcard_inserted(0));
#endif /* CONFIG_CXD56_SDCARD_AUTOMOUNT */

      g_sdhci.initialized = false;
    }
}

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
/****************************************************************************
 * Name: board_sdcard_detect_int
 *
 * Description:
 *   Card detect interrupt handler
 *
 ****************************************************************************/

static int board_sdcard_detect_int(int irq, void *context, void *arg)
{
  bool inserted;

  /* Get the state of the GPIO pin */

  inserted = board_sdcard_inserted(0);

  /* Has the card detect state changed? */

  if (inserted != g_sdhci.inserted)
    {
      /* Yes... remember that new state and inform the SDHCI driver */

      g_sdhci.inserted = inserted;

      if (inserted)
        {
          /* Card Detect = Present, Write Protect = disable */

          putreg32(0, CXD56_TOPREG_IOFIX_APP);
        }
      else
        {
          /* Card Detect = Not present, Write Protect = disable */

          putreg32(1, CXD56_TOPREG_IOFIX_APP);
        }

      /* Check context */

      if (up_interrupt_context())
        {
          work_cancel(HPWORK, &g_sdcard_work);
          if (inserted)
            {
              work_queue(HPWORK, &g_sdcard_work, board_sdcard_enable,
                         NULL, 0);
            }
          else
            {
              work_queue(HPWORK, &g_sdcard_work, board_sdcard_disable,
                         NULL, 0);
            }
        }
      else
        {
          if (inserted)
            {
              board_sdcard_enable(NULL);
            }
          else
            {
              board_sdcard_disable(NULL);
            }
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sdcard_initialize
 *
 * Description:
 *   Initialize SD Card on the board.
 *
 ****************************************************************************/

int board_sdcard_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_SDCARD_TXS02612_PORT0
  /* Select port0 for SD-Card (default) */

#else
  /* Select port1 for SDIO other than SD-Card */

  cxd56_gpio_config(SDCARD_TXS02612_SEL, false);
  cxd56_gpio_write(SDCARD_TXS02612_SEL, true);
#endif

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  /* Initialize Card insert status */

  g_sdhci.inserted = false;

  /* Configure Interrupt pin with internal pull-up */

  cxd56_pin_config(PINCONF_SDIO_CD_GPIO);
  cxd56_gpioint_config(PIN_SDIO_CD, GPIOINT_PSEUDO_EDGE_BOTH,
                       board_sdcard_detect_int, NULL);

  /* Handle the case when SD card is already inserted */

  board_sdcard_detect_int(PIN_SDIO_CD, NULL, NULL);

  /* Enabling Interrupt */

  cxd56_gpioint_enable(PIN_SDIO_CD);
#else
  /* Enable SDC */

  board_sdcard_enable(NULL);
#endif

  return ret;
}

/****************************************************************************
 * Name: board_sdcard_finalize
 *
 * Description:
 *   Finalize SD Card on the board.
 *
 ****************************************************************************/

int board_sdcard_finalize(void)
{
  int ret = OK;

  /* At first, Disable interrupt of the card detection */

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  /* Disabling Interrupt */

  cxd56_gpioint_disable(PIN_SDIO_CD);

  g_sdhci.inserted = false;
#endif

  board_sdcard_disable(NULL);

  /* Disable SDIO pin configuration */

  CXD56_PIN_CONFIGS(PINCONFS_SDIOA_GPIO);

  /* Set GPIO pin to initial state */

  cxd56_gpio_write(PIN_SDIO_CLK, false);
  cxd56_gpio_write(PIN_SDIO_CMD, false);
  cxd56_gpio_write(PIN_SDIO_DATA0, false);
  cxd56_gpio_write(PIN_SDIO_DATA1, false);
  cxd56_gpio_write(PIN_SDIO_DATA2, false);
  cxd56_gpio_write(PIN_SDIO_DATA3, false);
  cxd56_gpio_write_hiz(SDCARD_TXS02612_SEL);

  return ret;
}

/****************************************************************************
 * Name: board_sdcard_pin_initialize
 *
 * Description:
 *   Initialize SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_initialize(void)
{
}

/****************************************************************************
 * Name: board_sdcard_pin_finalize
 *
 * Description:
 *   Finalize SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_finalize(void)
{
}

/****************************************************************************
 * Name: board_sdcard_pin_configuraton
 *
 * Description:
 *   Configure SD Card pins on the board.
 *   This is called when SDHCI is used.
 *
 ****************************************************************************/

void board_sdcard_pin_configuraton(void)
{
  /* SDIO configuration */

  modifyreg32(CXD56_SDHCI_USERDEF1CTL, SDHCI_UDEF1_SDCLKI_SEL,
              SDHCI_UDEF1_SDCLKI_SEL_INT);
  modifyreg32(CXD56_SDHCI_USERDEF2CTL, SDHCI_UDEF2_CMD_SEL,
              SDHCI_UDEF2_CMD_SEL_INT);

  /* Disable GPIO output */

  cxd56_gpio_write_hiz(PIN_SDIO_CLK);
  cxd56_gpio_write_hiz(PIN_SDIO_CMD);
  cxd56_gpio_write_hiz(PIN_SDIO_DATA0);
  cxd56_gpio_write_hiz(PIN_SDIO_DATA1);
  cxd56_gpio_write_hiz(PIN_SDIO_DATA2);
  cxd56_gpio_write_hiz(PIN_SDIO_DATA3);

  /* SDIO pin configuration */

  CXD56_PIN_CONFIGS(PINCONFS_SDIOA_SDIO);
}

/****************************************************************************
 * Name: board_sdcard_pin_enable
 *
 * Description:
 *   Enable SD Card on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_enable(void)
{
}

/****************************************************************************
 * Name: board_sdcard_pin_disable
 *
 * Description:
 *   Disable SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_disable(void)
{
}

/****************************************************************************
 * Name: board_sdcard_set_high_voltage
 *
 * Description:
 *   Set SD Card IO voltage to 3.3V
 *
 ****************************************************************************/

void board_sdcard_set_high_voltage(void)
{
}

/****************************************************************************
 * Name: board_sdcard_set_low_voltage
 *
 * Description:
 *   Set SD Card IO voltage to 1.8V
 *
 ****************************************************************************/

void board_sdcard_set_low_voltage(void)
{
}

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
/****************************************************************************
 * Name: board_sdcard_inserted
 *
 * Description:
 *   Check if a card is inserted into the selected SDHCI slot
 *
 ****************************************************************************/

bool board_sdcard_inserted(int slotno)
{
  bool removed;

  /* Get the state of the GPIO pin */

  removed = cxd56_gpio_read(PIN_SDIO_CD);
  finfo("Slot %d inserted: %s\n", slotno, removed ? "NO" : "YES");

  return !removed;
}
#endif

/****************************************************************************
 * Name: board_sdcard_set_state_cb
 *
 * Description:
 *   Register callback function to notify state change of card slot.
 *   This function is called by board_ioctl()
 *    as BOARDIOC_SDCARD_SETNOTIFYCB command.
 *
 ****************************************************************************/

int board_sdcard_set_state_cb(uintptr_t cb)
{
  if (g_sdhci.cb != NULL && cb != 0)
    {
      return -EBUSY;
    }

  g_sdhci.cb = (void (*)(bool))cb;
  return OK;
}
