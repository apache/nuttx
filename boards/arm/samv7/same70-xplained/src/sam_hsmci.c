/****************************************************************************
 * config/same70-xplained/src/sam_hsmci.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 *
 ****************************************************************************/

/* The SAM E70 Xplained Ultraas one standard SD card connector that is
 * connected to the High Speed Multimedia Card Interface (HSMCI) of the SAM
 * E70. SD card connector:
 *
 *   ------ ----------------- ---------------------
 *   SAME70 SAME70            Shared functionality
 *   Pin    Function
 *   ------ ----------------- ---------------------
 *   PA30   MCDA0 (DAT0)
 *   PA31   MCDA1 (DAT1)
 *   PA26   MCDA2 (DAT2)
 *   PA27   MCDA3 (DAT3)      Camera
 *   PA25   MCCK (CLK)        Shield
 *   PA28   MCCDA (CMD)
 *   PC16   Card Detect (C/D) Shield
 *   ------ ----------------- ---------------------
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

#include "sam_gpio.h"
#include "sam_hsmci.h"

#include "same70-xplained.h"

#ifdef HAVE_HSMCI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure holds static information unique to one HSMCI peripheral */

struct sam_hsmci_state_s
{
  struct sdio_dev_s *hsmci;   /* R/W device handle */
  gpio_pinset_t cdcfg;        /* Card detect PIO pin configuration */
  gpio_pinset_t pwrcfg;       /* Power PIO pin configuration */
  uint8_t irq;                /* Interrupt number (same as pid) */
  uint8_t slotno;             /* Slot number */
  bool cd;                    /* TRUE: card is inserted */
  xcpt_t handler;             /* Interrupt handler */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HSCMI device state */

#ifdef CONFIG_SAMV7_HSMCI0
static int sam_hsmci0_cardetect(int irq, void *regs, FAR void *arg);

static struct sam_hsmci_state_s g_hsmci0 =
{
  .cdcfg   = GPIO_MCI0_CD,
  .irq     = IRQ_MCI0_CD,
  .slotno  = 0,
  .handler = sam_hsmci0_cardetect,
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

  inserted = sam_gpioread(state->cdcfg);
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

#ifdef CONFIG_SAMV7_HSMCI0
static int sam_hsmci0_cardetect(int irq, void *regs, FAR void *arg)
{
  int ret;

  /* Handle the card detect interrupt.  The interrupt level logic will
   * kick of the driver-level operations to initialize the MMC/SD block
   * device.
   */

  ret = sam_hsmci_cardetect(&g_hsmci0);

#ifdef CONFIG_SAME70XPLAINED_HSMCI0_AUTOMOUNT
  /* Let the automounter know about the insertion event */

  sam_automount_event(HSMCI0_SLOTNO, sam_cardinserted(HSMCI0_SLOTNO));
#endif

  return ret;
}
#endif

/****************************************************************************
 * Name: sam_hsmci_state
 *
 * Description:
 *   Initialize HSMCI PIOs.
 *
 ****************************************************************************/

static inline struct sam_hsmci_state_s *sam_hsmci_state(int slotno)
{
  struct sam_hsmci_state_s *state = NULL;

#ifdef CONFIG_SAMV7_HSMCI0
  state = &g_hsmci0;
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

  /* Initialize card-detect, write-protect, and power enable PIOs */

  sam_configgpio(state->cdcfg);
  sam_dumpgpio(state->cdcfg, "HSMCI Card Detect");

  if (state->pwrcfg != 0)
    {
      sam_configgpio(state->pwrcfg);
      sam_dumpgpio(state->pwrcfg, "HSMCI Power");
    }

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

  sam_gpioirq(state->cdcfg);
  (void)irq_attach(state->irq, state->handler, NULL);

  /* Then inform the HSMCI driver if there is or is not a card in the slot. */

  state->cd = sam_cardinserted_internal(state);
  sdio_mediachange(state->hsmci, state->cd);

  /* Enable card detect interrupts */

  sam_gpioirqenable(state->irq);
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
