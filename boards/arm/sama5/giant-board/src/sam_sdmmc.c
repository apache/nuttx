/****************************************************************************
 * boards/arm/sama5/giant-board/src/sam_sdmmc.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/* The Giant Board provide one Micro SD memory card slot at J6 (SDMMC1).
 *
 * SDMMC1: The Micro SD card slot connects via SDMMC1.  The card detect
 * discrete is available on PA21 (pulled high), shared with DAT3. The write
 * protect discrete is tied to ground and not available to software. The
 * slot only supports 4-bit wide transfer mode.
 *
 *   PA18  SDMMC1_DAT0
 *   PA19  SDMMC1_DAT1
 *   PA20  SDMMC1_DAT2
 *   PA21  SDMMC1_DAT3 / CD
 *   PD22  SDMMC1_CK
 *   PA28  SDMMC1_CMD
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "chip.h"
#include "arm_internal.h"
#include "sam_pio.h"
#include "sam_sdmmc.h"

#include "giant-board.h"

#ifdef HAVE_SDMMC

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds static information unique to one SDMMC peripheral */

struct sam_sdmmc_state_s
{
  struct sdio_dev_s *sdmmc;   /* R/W device handle */
  pio_pinset_t pincfg;        /* Card detect PIO pin configuration */
  uint8_t irq;                /* Interrupt number (same as pid) */
  uint8_t slotno;             /* Slot number */
  bool cd;                    /* TRUE: card is inserted */
  xcpt_t handler;             /* Interrupt handler */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SDMMC device state */

#ifdef CONFIG_SAMA5_SDMMC0
static int sam_sdmmc0_cardetect(int irq, void *regs, void *arg);

static struct sam_sdmmc_state_s g_sdmmc0 =
{
  .pincfg  = PIO_SDMMC0_CD,
  .irq     = -1,
  .slotno  = 0,
  .handler = sam_sdmmc0_cardetect,
};
#endif

#ifdef CONFIG_SAMA5_SDMMC1
static int sam_sdmmc1_cardetect(int irq, void *regs, void *arg);

static struct sam_sdmmc_state_s g_sdmmc1 =
{
  .pincfg  = PIO_SDMMC1_CD,
  .irq     = IRQ_SDMMC1_CD,
  .slotno  = 1,
  .handler = sam_sdmmc1_cardetect,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_cardinserted_internal
 *
 * Description:
 *   Check if a card is inserted into the selected SDMMC slot
 *
 ****************************************************************************/

bool sam_cardinserted_internal(struct sam_sdmmc_state_s *state)
{
  bool inserted;

  /* Get the state of the PIO pin */

  inserted = sam_pioread(state->pincfg);
  finfo("Slot %d inserted: %s\n", state->slotno, inserted ? "NO" : "YES");
  return !inserted;
}

/****************************************************************************
 * Name: sam_sdmmc_cardetect, sam_sdmmc0_cardetect, and sam_sdmmc1_cardetect
 *
 * Description:
 *   Card detect interrupt handlers
 *
 ****************************************************************************/

static int sam_sdmmc_cardetect(struct sam_sdmmc_state_s *state)
{
  /* Get the current card insertion state */

  bool cd = sam_cardinserted_internal(state);

  /* Has the card detect state changed? */

  if (cd != state->cd)
    {
      /* Yes... remember that new state and inform the SDMMC driver */

      state->cd = cd;

      /* Report the new state to the SDIO driver */

      sdio_mediachange(state->sdmmc, cd);
    }

  return OK;
}

#ifdef CONFIG_SAMA5_SDMMC0
static int sam_sdmmc0_cardetect(int irq, void *regs, void *arg)
{
  return sam_sdmmc_cardetect(&g_sdmmc0);
}
#endif

#ifdef CONFIG_SAMA5_SDMMC1
static int sam_sdmmc1_cardetect(int irq, void *regs, void *arg)
{
  return sam_sdmmc_cardetect(&g_sdmmc1);
}
#endif

/****************************************************************************
 * Name: sam_sdmmc_state
 *
 * Description:
 *   Initialize SDMMC PIOs.
 *
 ****************************************************************************/

static struct sam_sdmmc_state_s *sam_sdmmc_state(int slotno)
{
  struct sam_sdmmc_state_s *state = NULL;

#ifdef CONFIG_SAMA5_SDMMC0
#ifdef CONFIG_SAMA5_SDMMC1
  if (slotno == 0)
#endif
    {
      state = &g_sdmmc0;
    }
#ifdef CONFIG_SAMA5_SDMMC1
  else
#endif
#endif

#ifdef CONFIG_SAMA5_SDMMC1
    {
      state = &g_sdmmc1;
    }
#endif

  return state;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdmmc_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int sam_sdmmc_initialize(int slotno, int minor)
{
  struct sam_sdmmc_state_s *state;
  int ret;

  mcinfo("Entry.\n");
  mcinfo("slotno: %d\n", slotno);

  /* Get the static SDMMC description */

  state = sam_sdmmc_state(slotno);
  if (!state)
    {
      ferr("ERROR: No state for slotno %d\n", slotno);
      return -EINVAL;
    }

  /* Initialize card-detect and write-protect PIOs */

  sam_configpio(state->pincfg);

  /* Mount the SDIO-based MMC/SD block driver */

  /* First, get an instance of the SDIO interface */

  state->sdmmc = sam_sdmmc_sdio_initialize(slotno);
  if (!state->sdmmc)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n",  slotno);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(minor, state->sdmmc);
  if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  /* Configure card detect interrupts */

  sam_pioirq(state->pincfg);
  irq_attach(state->irq, state->handler, NULL);

  /* Then inform the SDMMC driver if there is or is not a card in the slot. */

  state->cd = sam_cardinserted_internal(state);
  sdio_mediachange(state->sdmmc, state->cd);

  /* Enable card detect interrupts */

  sam_pioirqenable(state->irq);
  return OK;
}

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected SDMMC slot
 *
 ****************************************************************************/

bool sam_cardinserted(int slotno)
{
  struct sam_sdmmc_state_s *state;

  /* Get the SDMMC description */

  state = sam_sdmmc_state(slotno);
  if (!state)
    {
      ferr("ERROR: No state for slotno %d\n", slotno);
      return false;
    }

  /* Return the state of the PIO pin */

  return sam_cardinserted_internal(state);
}

/****************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the selected SDMMC slot
 *
 ****************************************************************************/

bool sam_writeprotected(int slotno)
{
  /* There are no write protect pins */

  return false;
}

#endif /* HAVE_SDMMC */

