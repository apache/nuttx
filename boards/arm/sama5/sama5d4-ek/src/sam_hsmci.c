/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/sam_hsmci.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

/* The SAMA5D4-EK provides a two SD memory card slots:  (1) a full size SD
 * card slot (J10), and (2) a microSD memory card slot (J11).
 *
 * The full size SD card slot connects via HSMCI0.  The card detect discrete
 * is available on PE5 (pulled high).  The write protect discrete is tied to
 * ground and is not available to software.  The slot supports 8-bit wide
 * transfer mode, but the NuttX driver currently uses only the 4-bit wide
 * transfer mode
 *
 * ------------------------------ ------------------- -----------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -----------------------
 * PC4/SPI0_NPCS1/MCI0_CK/PCK1    PC4                 MCI0_CK, ISI_MCK, EXP
 * PC5/D0/MCI0_CDA                PC5                 MCI0_CDA, NAND_IO0
 * PC6/D1/MCI0_DA0                PC6                 MCI0_DA0, NAND_IO1
 * PC7/D2/MCI0_DA1                PC7                 MCI0_DA1, NAND_IO2
 * PC8/D3/MCI0_DA2                PC8                 MCI0_DA2, NAND_IO3
 * PC9/D4/MCI0_DA3                PC9                 MCI0_DA3, NAND_IO4
 * PC10/D5/MCI0_DA4               PC10                MCI0_DA4, NAND_IO5
 * PC11/D6/MCI0_DA5               PC11                MCI0_DA5, NAND_IO6
 * PC12/D7/MCI0_DA6               PC12                MCI0_DA6, NAND_IO7
 * PC13/NRD/NANDOE/MCI0_DA7       PC13                MCI0_DA7, NAND_RE
 * PE5/A5/CTS3                    MCI0_CD_PE5         MCI0_CD
 * ------------------------------ ------------------- -----------------------
 *
 * The microSD connects vi HSMCI1.  The card detect discrete is available on
 * PE6 (pulled high):
 *
 * ------------------------------ ------------------- -----------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -----------------------
 * PE14/A14/TCLK1/PWMH3           MCI1_CD_PE14        MCI1_CD             ???
 * PE15/A15/SCK3/TIOA0            MCI1_PWR_PE15       MCI1_PWR
 * PE18/A18/TIOA5/MCI1_CK         PE18                MCI1_CK, EXP
 * PE19/A19/TIOB5/MCI1_CDA        PE19                MCI1_CDA, EXP
 * PE20/A20/TCLK5/MCI1_DA0        PE20                MCI1_DA0, EXP
 * PE21/A23/TIOA4/MCI1_DA1        PE21                MCI1_DA1, EXP
 * PE22/A24/TIOB4/MCI1_DA2        PE22                MCI1_DA2, EXP
 * PE23/A25/TCLK4/MCI1_DA3        PE23                MCI1_DA3, EXP
 * PE6/A6/TIOA3                   MCI1_CD_PE6         MCI1_CD
 * ------------------------------ ------------------- -----------------------
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

#include "sama5d4-ek.h"

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
  pio_pinset_t cdcfg;         /* Card detect PIO pin configuration */
  pio_pinset_t pwrcfg;        /* Power PIO pin configuration */
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
static int sam_hsmci0_cardetect(int irq, void *regs, FAR void *arg);

static struct sam_hsmci_state_s g_hsmci0 =
{
  .cdcfg   = PIO_MCI0_CD,
  .irq     = IRQ_MCI0_CD,
  .slotno  = 0,
  .handler = sam_hsmci0_cardetect,
};
#endif

#ifdef CONFIG_SAMA5_HSMCI1
static int sam_hsmci1_cardetect(int irq, void *regs, FAR void *arg);

static struct sam_hsmci_state_s g_hsmci1 =
{
  .cdcfg   = PIO_MCI1_CD,
  .pwrcfg  = IRQ_MCI1_PWR,
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

  inserted = sam_pioread(state->cdcfg);
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
static int sam_hsmci0_cardetect(int irq, FAR void *regs, FAR void *arg)
{
  int ret;

  /* Handle the card detect interrupt.  The interrupt level logic will
   * kick of the driver-level operations to initialize the MMC/SD block
   * device.
   */

  ret = sam_hsmci_cardetect(&g_hsmci0);

#ifdef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT
  /* Let the automounter know about the insertion event */

  sam_automount_event(HSMCI0_SLOTNO, sam_cardinserted(HSMCI0_SLOTNO));
#endif

  return ret;
}
#endif

#ifdef CONFIG_SAMA5_HSMCI1
static int sam_hsmci1_cardetect(int irq, FAR void *regs, FAR void *arg)
{
  int ret;

  /* Handle the card detect interrupt.  The interrupt level logic will
   * kick of the driver-level operations to initialize the MMC/SD block
   * device.
   */

  ret = sam_hsmci_cardetect(&g_hsmci1);

#ifdef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT
  /* Let the automounter know about the insertion event */

  sam_automount_event(HSMCI1_SLOTNO, sam_cardinserted(HSMCI1_SLOTNO));
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

  /* Initialize card-detect, write-protect, and power enable PIOs */

  sam_configpio(state->cdcfg);
  sam_dumppio(state->cdcfg, "HSMCI Card Detect");

  if (state->pwrcfg != 0)
    {
      sam_configpio(state->pwrcfg);
      sam_dumppio(state->pwrcfg, "HSMCI Power");
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

  sam_pioirq(state->cdcfg);
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
