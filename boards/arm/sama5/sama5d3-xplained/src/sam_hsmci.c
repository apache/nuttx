/****************************************************************************
 * boards/arm/sama5/sama5d3-xplained/src/sam_hsmci.c
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

/* The SAMA5D3-Xplained provides a two SD memory card slots:
 *  (1) a full size SD card slot (J10), and
 *  (2) a microSD memory card slot (J11).
 *
 * The full size SD card slot connects via HSMCI0.  The card detect discrete
 * is available on PB17 (pulled high).  The write protect descrete is tied to
 * ground (via PP6) and not available to software.  The slot supports 8-bit
 * wide transfer mode, but the NuttX driver currently uses only the 4-bit
 * wide transfer mode
 *
 *   PD17 MCI0_CD
 *   PD1  MCI0_DA0
 *   PD2  MCI0_DA1
 *   PD3  MCI0_DA2
 *   PD4  MCI0_DA3
 *   PD5  MCI0_DA4
 *   PD6  MCI0_DA5
 *   PD7  MCI0_DA6
 *   PD8  MCI0_DA7
 *   PD9  MCI0_CK
 *   PD0  MCI0_CDA
 *
 * The microSD connects vi HSMCI1.  The card detect discrete is available on
 * PB18 (pulled high):
 *
 *   PD18  MCI1_CD
 *   PB20  MCI1_DA0
 *   PB21  MCI1_DA1
 *   PB22  MCI1_DA2
 *   PB23  MCI1_DA3
 *   PB24  MCI1_CK
 *   PB19  MCI1_CDA
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

#include "sam_pio.h"
#include "sam_hsmci.h"

#include "sama5d3-xplained.h"

#ifdef HAVE_HSMCI

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds static information unique to one HSMCI peripheral */

struct sam_hsmci_state_s
{
  struct sdio_dev_s *hsmci;   /* R/W device handle */
  pio_pinset_t pincfg;        /* Card detect PIO pin configuration */
  uint8_t irq;                /* Interrupt number (same as pid) */
  uint8_t slotno;             /* Slot number */
  bool cd;                    /* TRUE: card is inserted */
  xcpt_t handler;             /* Interrupt handler */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HSCMI device state */

#ifdef CONFIG_SAMA5_HSMCI0
static int sam_hsmci0_cardetect(int irq, void *regs, void *arg);

static struct sam_hsmci_state_s g_hsmci0 =
{
  .pincfg  = PIO_MCI0_CD,
  .irq     = IRQ_MCI0_CD,
  .slotno  = 0,
  .handler = sam_hsmci0_cardetect,
};
#endif

#ifdef CONFIG_SAMA5_HSMCI1
static int sam_hsmci1_cardetect(int irq, void *regs, void *arg);

static struct sam_hsmci_state_s g_hsmci1 =
{
  .pincfg  = PIO_MCI1_CD,
  .irq     = IRQ_MCI1_CD,
  .slotno  = 1,
  .handler = sam_hsmci1_cardetect,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_cardinserted_internal
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

bool sam_cardinserted_internal(struct sam_hsmci_state_s *state)
{
  bool inserted;

  /* Get the state of the PIO pin */

  inserted = sam_pioread(state->pincfg);
  finfo("Slot %d inserted: %s\n", state->slotno, inserted ? "NO" : "YES");
  return !inserted;
}

/****************************************************************************
 * Name: sam_hsmci_cardetect, sam_hsmci0_cardetect, and sam_hsmci1_cardetect
 *
 * Description:
 *   Card detect interrupt handlers
 *
 ****************************************************************************/

static int sam_hsmci_cardetect(struct sam_hsmci_state_s *state)
{
  /* Get the current card insertion state */

  bool cd = sam_cardinserted_internal(state);

  /* Has the card detect state changed? */

  if (cd != state->cd)
    {
      /* Yes... remember that new state and inform the HSMCI driver */

      state->cd = cd;

      /* Report the new state to the SDIO driver */

      sdio_mediachange(state->hsmci, cd);
    }

  return OK;
}

#ifdef CONFIG_SAMA5_HSMCI0
static int sam_hsmci0_cardetect(int irq, void *regs, void *arg)
{
  return sam_hsmci_cardetect(&g_hsmci0);
}
#endif

#ifdef CONFIG_SAMA5_HSMCI1
static int sam_hsmci1_cardetect(int irq, void *regs, void *arg)
{
  return sam_hsmci_cardetect(&g_hsmci1);
}
#endif

/****************************************************************************
 * Name: sam_hsmci_state
 *
 * Description:
 *   Initialize HSMCI PIOs.
 *
 ****************************************************************************/

static struct sam_hsmci_state_s *sam_hsmci_state(int slotno)
{
  struct sam_hsmci_state_s *state = NULL;

#ifdef CONFIG_SAMA5_HSMCI0
#ifdef CONFIG_SAMA5_HSMCI1
  if (slotno == 0)
#endif
    {
      state = &g_hsmci0;
    }
#ifdef CONFIG_SAMA5_HSMCI1
  else
#endif
#endif

#ifdef CONFIG_SAMA5_HSMCI1
    {
      state = &g_hsmci1;
    }
#endif

  return state;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int sam_hsmci_initialize(int slotno, int minor)
{
  struct sam_hsmci_state_s *state;
  int ret;

  /* Get the static HSMI description */

  state = sam_hsmci_state(slotno);
  if (!state)
    {
      ferr("ERROR: No state for slotno %d\n", slotno);
      return -EINVAL;
    }

  /* Initialize card-detect and write-protect PIOs */

  sam_configpio(state->pincfg);

  /* Mount the SDIO-based MMC/SD block driver */

  /* First, get an instance of the SDIO interface */

  state->hsmci = sdio_initialize(slotno);
  if (!state->hsmci)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n",  slotno);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(minor, state->hsmci);
  if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  /* Configure card detect interrupts */

  sam_pioirq(state->pincfg);
  irq_attach(state->irq, state->handler, NULL);

  /* Then inform the HSMCI driver if there is or is not a card in the slot. */

  state->cd = sam_cardinserted_internal(state);
  sdio_mediachange(state->hsmci, state->cd);

  /* Enable card detect interrupts */

  sam_pioirqenable(state->irq);
  return OK;
}

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

bool sam_cardinserted(int slotno)
{
  struct sam_hsmci_state_s *state;

  /* Get the HSMI description */

  state = sam_hsmci_state(slotno);
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
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

bool sam_writeprotected(int slotno)
{
  /* There are no write protect pins */

  return false;
}

#endif /* HAVE_HSMCI */
