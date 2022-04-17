/****************************************************************************
 * boards/arm/stm32/clicker2-stm32/src/stm32_mmcsd.c
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

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>
#include <nuttx/wqueue.h>

#include "stm32_spi.h"

#include "clicker2-stm32.h"

#ifdef CONFIG_MMCSD_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_CLICKER2_STM32_MB1_MMCSD) && \
    !defined(CONFIG_CLICKER2_STM32_MB2_MMCSD)
#  error Only the Mikroe uSD click boards are supported
#endif

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  error Mountpoints are required for MMCSD support
#endif

#ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD
#  ifndef CONFIG_STM32_SPI3
#    error MMCSD on mikroBUS1 requires CONFIG_STM32_SPI3
#  endif
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD
#  ifndef CONFIG_STM32_SPI2
#    error MMCSD on mikroBUS1 requires CONFIG_STM32_SPI2
#  endif
#endif

#ifdef CONFIG_SCHED_LPWORK
#  define MMCSDWORK LPWORK
#elif defined (CONFIG_SCHED_HPWORK)
#  define MMCSDWORK HPWORK
#else
#  error High or low priority work queue required for MMCSD support
#endif

/* Card Detect
 *
 *   mikroBUS1 Card Detect (AN pin): PE10-MB1_INT
 *   mikroBUS2 Card Detect (AN pin:  PE14-MB2_INT
 *
 * There is a pull-up on the uSD click board`
 */

#define GPIO_MB1_CD   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN2)
#define GPIO_MB2_CD   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds static information unique to one MMCSD slot */

struct stm32_mmcsd_state_s
{
  uint8_t             spidev;   /* SPI bus used for MMCSD */
  uint8_t             slotno;   /* Slot number */
  int                 minor;    /* The MMC/SD minor device number */
  uint32_t            cdcfg;    /* Card detect PIO pin configuration */
  xcpt_t              handler;  /* Interrupt handler */
  bool                cd;       /* TRUE: card is inserted */
  spi_mediachange_t   callback; /* SPI media change callback */
  void                *cbarg;   /* Argument to pass to media change callback */
  struct work_s       work;     /* For deferring card detect interrupt work */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool stm32_cardinserted_internal(struct stm32_mmcsd_state_s *state);
static void stm32_mmcsd_carddetect(void *arg);
static int stm32_mmcsd_setup(struct stm32_mmcsd_state_s *);

#ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD
static int stm32_mb1_mmcsd_carddetect(int irq,
                                      void *regs,
                                      void *arg);
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD
static int stm32_mb2_mmcsd_carddetect(int irq,
                                      void *regs,
                                      void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* MMCSD device state */

#ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD
static int stm32_mb1_mmcsd_carddetect(int irq, void *regs, void *arg);

static struct stm32_mmcsd_state_s g_mb1_mmcsd =
{
  .spidev    = 3,
  .slotno    = MB1_MMCSD_SLOTNO,
  .minor     = MB1_MMCSD_MINOR,
  .cdcfg     = GPIO_MB1_CD,
  .handler   = stm32_mb1_mmcsd_carddetect,
  .callback  = NULL,
  .cbarg     = NULL,
};
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD
static int stm32_mb2_mmcsd_carddetect(int irq, void *regs, void *arg);

static struct stm32_mmcsd_state_s g_mb2_mmcsd =
{
  .spidev    = 2,
  .slotno    = MB2_MMCSD_SLOTNO,
  .minor     = MB2_MMCSD_MINOR,
  .cdcfg     = GPIO_MB2_CD,
  .handler   = stm32_mb2_mmcsd_carddetect,
  .callback  = NULL,
  .cbarg     = NULL,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_cardinserted_internal
 *
 * Description:
 *   Check if a card is inserted into the selected MMCSD slot
 *
 ****************************************************************************/

static bool stm32_cardinserted_internal(struct stm32_mmcsd_state_s *state)
{
  bool inserted;

  /* Get the state of the PIO pin */

  inserted = stm32_gpioread(state->cdcfg);
  finfo("Slot %d inserted: %s\n", state->slotno, inserted ? "NO" : "YES");
  return !inserted;
}

/****************************************************************************
 * Name: stm32_mmcsd_carddetect, stm32_mb1_mmcsd_carddetect, and
 *       stm32_mb2_mmcsd_carddetect
 *
 * Description:
 *   Card detect interrupt handlers
 *
 ****************************************************************************/

static void stm32_mmcsd_carddetect(void *arg)
{
  bool cd;
  struct stm32_mmcsd_state_s *state =
                                   (struct stm32_mmcsd_state_s *)arg;

  /* Get the current card insertion state */

  cd = stm32_cardinserted_internal(state);

  /* Has the card detect state changed? */

  if (cd != state->cd)
    {
      /* Yes... remember that new state and inform the HSMCI driver */

      state->cd = cd;

      /* Report the new state to the SPI driver */

      if (state->callback)
        {
          state->callback(state->cbarg);
        }
    }

#ifdef HAVE_AUTOMOUNTER
  /* Let the automounter know about the insertion event */

  stm32_automount_event(state->slotno, stm32_cardinserted(state->slotno));
#endif
}

#ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD
static int stm32_mb1_mmcsd_carddetect(int irq, void *regs, void *arg)
{
  if (work_available(&g_mb1_mmcsd.work))
    {
      return work_queue(MMCSDWORK, &g_mb1_mmcsd.work, stm32_mmcsd_carddetect,
                        &g_mb1_mmcsd, 0);
    }

  return OK;
}
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD
static int stm32_mb2_mmcsd_carddetect(int irq, void *regs, void *arg)
{
  if (work_available(&g_mb2_mmcsd.work))
    {
      return work_queue(MMCSDWORK, &g_mb2_mmcsd.work, stm32_mmcsd_carddetect,
                        &g_mb2_mmcsd, 0);
    }

  return OK;
}
#endif

static int stm32_mmcsd_setup(struct stm32_mmcsd_state_s *state)
{
  struct spi_dev_s *spi;
  int ret;

  /* Initialize the SPI bus and get an instance of the SPI interface */

  spi = stm32_spibus_initialize(state->spidev);
  if (spi == NULL)
    {
      spierr("ERROR: Failed to initialize SPI bus %d\n", state->spidev);
      return -ENODEV;
    }

  ret = mmcsd_spislotinitialize(state->minor, state->slotno, spi);
  if (ret < 0)
    {
      mcerr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
            state->spidev, state->slotno);
      return ret;
    }

  /* Initialize Card Detect pin and enable interrupt on edges */

  stm32_configgpio(state->cdcfg);
  stm32_gpiosetevent(state->cdcfg, true, true, true, state->handler, NULL);

  state->cd =  stm32_cardinserted_internal(state);
  if (state->callback)
    {
      state->callback(state->cbarg);
    }

#ifdef HAVE_AUTOMOUNTER
  /* Let the automounter know about the insertion event */

  stm32_automount_event(state->slotno, stm32_cardinserted(state->slotno));
#endif

  mcinfo("INFO: mmcsd%d card has been initialized successfully\n",
         state->minor);
  return OK;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initialize the MMCSD device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_mmcsd_initialize(void)
{
  int ret;

#ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD
  finfo("Configuring MMCSD on mikroBUS1\n");

  ret = stm32_mmcsd_setup(&g_mb1_mmcsd);
  if (ret < 0)
    {
      mcerr("ERROR: Failed to initialize MMCSD on mikroBus1: %d\n", ret);
    }
#endif

#ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD
  finfo("Configuring MMCSD on mikroBUS2\n");
  ret = stm32_mmcsd_setup(&g_mb2_mmcsd);
  if (ret < 0)
    {
      mcerr("ERROR: Failed to initialize MMCSD on mikroBus2: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}

/****************************************************************************
 * Name: stm32_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected MMCSD slot
 *
 ****************************************************************************/

bool stm32_cardinserted(int slotno)
{
  struct stm32_mmcsd_state_s *state;

  /* Get the MMCSD description */

#ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD
  if (slotno == g_mb1_mmcsd.slotno)
    {
      state = &g_mb1_mmcsd;
    }
#endif
#ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD

  if (slotno == g_mb2_mmcsd.slotno)
    {
      state = &g_mb2_mmcsd;
    }
#endif

  if (!state)
    {
      ferr("ERROR: No state for slotno %d\n", slotno);
      return false;
    }

  /* Return the state of the CD pin */

  return stm32_cardinserted_internal(state);
}

/****************************************************************************
 * Name: stm32_spi2register
 *
 * Description:
 *   Registers media change callback
 ****************************************************************************/

int stm32_spi2register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg)
{
  spiinfo("INFO: Registering spi2 device\n");
#ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD
  g_mb2_mmcsd.callback = callback;
  g_mb2_mmcsd.cbarg    = arg;
#endif
  return OK;
}

/****************************************************************************
 * Name: stm32_spi3register
 *
 * Description:
 *   Registers media change callback
 ****************************************************************************/

int stm32_spi3register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg)
{
  spiinfo("INFO: Registering spi3 device\n");
#ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD
  g_mb1_mmcsd.callback = callback;
  g_mb1_mmcsd.cbarg    = arg;
#endif
  return OK;
}

#endif /* CONFIG_MMCSD_SPI */
