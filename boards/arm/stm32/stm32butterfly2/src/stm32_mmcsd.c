/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_mmcsd.c
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

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/mmcsd.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include "stm32.h"
#include "stm32_butterfly2.h"
#include "stm32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_SPI1
#  error "SD driver requires CONFIG_STM32_SPI1 to be enabled"
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

static const int SD_SPI_PORT = 1; /* SD is connected to SPI1 port */
static const int SD_SLOT_NO = 0;  /* There is only one SD slot */

/* Media changed callback */

static spi_mediachange_t g_chmediaclbk;

/* Argument for media changed callback */

static void *g_chmediaarg;

/* Semafor to inform stm32_cd_thread that card was inserted or pulled out */

static sem_t g_cdsem;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_cd_thread
 *
 * Description:
 *   Working thread to call mediachanged function when card is inserted or
 *   pulled out.
 ****************************************************************************/

static void *stm32_cd_thread(void *arg)
{
  spiinfo("INFO: Running card detect thread\n");
  while (1)
    {
      nxsem_wait(&g_cdsem);
      spiinfo("INFO: Card has been inserted, initializing\n");

      if (g_chmediaclbk)
        {
          /* Card doesn't seem to initialize properly without letting it to
           * rest for a millisecond or so.
           */

          nxsig_usleep(1 * 1000);
          g_chmediaclbk(g_chmediaarg);
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: stm32_cd
 *
 * Description:
 *   Card detect interrupt handler.
 ****************************************************************************/

static int stm32_cd(int irq, void *context, void *arg)
{
  static const int debounce_time = 100; /* [ms] */
  static uint32_t now = 0;
  static uint32_t prev = 0;
  struct timespec tp;

  clock_systime_timespec(&tp);
  now = tp.tv_sec * 1000 + tp.tv_nsec / 1000000;

  /* When inserting card, card detect plate might bounce causing this
   * interrupt to be called many time on single card insert/deinsert. Thus
   * we are allowing only one interrupt every 100ms.
   */

  if (now - debounce_time > prev)
    {
      prev = now;
      nxsem_post(&g_cdsem);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spi1register
 *
 * Description:
 *   Registers media change callback
 ****************************************************************************/

int stm32_spi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg)
{
  spiinfo("INFO: Registering spi1 device\n");
  g_chmediaclbk = callback;
  g_chmediaarg = arg;
  return OK;
}

/****************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 ****************************************************************************/

int stm32_mmcsd_initialize(int minor)
{
  struct spi_dev_s *spi;
  struct sched_param schparam;
  pthread_attr_t pattr;
  int rv;

  spiinfo("INFO: Initializing mmcsd card\n");
  if ((spi = stm32_spibus_initialize(SD_SPI_PORT)) == NULL)
    {
      ferr("failed to initialize SPI port %d\n", SD_SPI_PORT);
      return -ENODEV;
    }

  if ((rv = mmcsd_spislotinitialize(minor, SD_SLOT_NO, spi)) < 0)
    {
      ferr("failed to bind SPI port %d to SD slot %d\n", SD_SPI_PORT,
           SD_SLOT_NO);
      return rv;
    }

  stm32_gpiosetevent(GPIO_SD_CD, true, true, true, stm32_cd, NULL);

  nxsem_init(&g_cdsem, 0, 0);
  pthread_attr_init(&pattr);

#ifdef CONFIG_DEBUG_FS
  pthread_attr_setstacksize(&pattr, 1024);
#else
  pthread_attr_setstacksize(&pattr, 256);
#endif

  schparam.sched_priority = 50;
  pthread_attr_setschedparam(&pattr, &schparam);
  pthread_create(NULL, &pattr, stm32_cd_thread, NULL);

  spiinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}
