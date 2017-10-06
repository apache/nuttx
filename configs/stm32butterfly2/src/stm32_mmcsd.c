/*****************************************************************************
 * configs/stm32butterfly2/src/stm32_mmcsd.c
 *
 *   Copyright (C) 2016-2017 Michał Łyszczek. All rights reserved.
 *   Author: Michał Łyszczek <michal.lyszczek@gmail.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 ****************************************************************************/

/*****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <time.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/mmcsd.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>

#include "stm32.h"
#include "stm32_butterfly2.h"
#include "stm32_spi.h"

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_SPI1
#  error "SD driver requires CONFIG_STM32_SPI1 to be enabled"
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

/*****************************************************************************
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

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: stm32_cd_thread
 *
 * Description:
 *   Working thread to call mediachanged function when card is inserted or
 *   pulled out.
 ****************************************************************************/

static void *stm32_cd_thread(void *arg)
{
  (void)arg;

  spiinfo("INFO: Runnig card detect thread\n");
  while (1)
    {
      nxsem_wait(&g_cdsem);
      spiinfo("INFO: Card has been inserted, initializing\n");

      if (g_chmediaclbk)
        {
          /* Card doesn't seem to initialize properly without letting it to
           * rest for a millsecond or so.
           */

          nxsig_usleep(1 * 1000);
          g_chmediaclbk(g_chmediaarg);
        }
    }

  return NULL;
}

/*****************************************************************************
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

  clock_gettime(CLOCK_MONOTONIC, &tp);
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

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
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

/*****************************************************************************
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

  (void)stm32_gpiosetevent(GPIO_SD_CD, true, true, true, stm32_cd, NULL);

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
