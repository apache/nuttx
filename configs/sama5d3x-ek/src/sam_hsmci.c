/****************************************************************************
 * config/sama5d3x-ek/src/sam_hsmci.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
/* The SAMA5D3x-EK provides a two SD memory card slots:  (1) a full size SD card
 * slot (J7 labeled MCI0), and (2) a microSD memory card slot (J6 labeled MCI1).
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

#include "sama5d3x-ek.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#define HAVE_MMCSD  1

/* Can't support MMC/SD if the card interface(s) are not enable */

#if !defined(CONFIG_SAMA5_HSMCI0) && !defined(CONFIG_SAMA5_HSMCI1)
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_MMCSD) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_MMCSD
#endif

/* We need PIO interrupts on PIOD to support card detect interrupts */

#if defined(HAVE_MMCSD) && !defined(CONFIG_SAMA5_PIOD_IRQ)
#  warning PIOD interrupts not enabled.  No MMC/SD support.
#  undef HAVE_MMCSD
#endif

/* The NSH slot and minor numbers are useless for us because we have
 * multiple HSMCI devices.
 */

#ifdef HAVE_MMCSD
#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    undef CONFIG_NSH_MMCSDSLOTNO
#  endif

#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif

#  ifndef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure holds information unique to one HSMCI peripheral */

struct sam_hsmci_info_s
{
  pio_pinset_t pincfg;        /* Card detect PIO pin configuratin */
  uint8_t irq;                /* Interrupt number (same as pid) */
  xcpt_t handler;             /* Interrupt handler */
  struct sdio_dev_s **hsmci;  /* R/W device handle */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Retained HSMCI driver handles for use by interrupt handlers */

#ifdef HAVE_MMCSD
#ifdef CONFIG_SAMA5_HSMCI0
static struct sdio_dev_s *g_hsmci0;
#endif
#ifdef CONFIG_SAMA5_HSMCI1
static struct sdio_dev_s *g_hsmci1;
#endif

/* HSCMI device characteristics */

#ifdef CONFIG_SAMA5_HSMCI0
static int sam_hsmci0_cardetect(int irq, void *regs);

static const struct sam_hsmci_info_s g_hsmci0_info =
{
  PIO_MCI0_CD, IRQ_MCI0_CD, sam_hsmci0_cardetect, &g_hsmci0
};
#endif

#ifdef CONFIG_SAMA5_HSMCI1
static int sam_hsmci1_cardetect(int irq, void *regs);

static const struct sam_hsmci_info_s g_hsmci1_info =
{
  PIO_MCI1_CD, IRQ_MCI1_CD, sam_hsmci1_cardetect, &g_hsmci1
};
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_hsmci0_cardetect and sam_hsmci1_cardetect
 *
 * Description:
 *   Card detect interrupt handlers
 *
 ****************************************************************************/

#ifdef HAVE_MMCSD
#ifdef CONFIG_SAMA5_HSMCI0
static int sam_hsmci0_cardetect(int irq, void *regs)
{
   sdio_mediachange(g_hsmci0, sam_cardinserted(0));
   return OK;
}
#endif

#ifdef CONFIG_SAMA5_HSMCI1
static int sam_hsmci1_cardetect(int irq, void *regs)
{
   sdio_mediachange(g_hsmci1, sam_cardinserted(1));
   return OK;
}
#endif
#endif

/****************************************************************************
 * Name: sam_hsmci_info
 *
 * Description:
 *   Initialize HSMCI PIOs.
 *
 ****************************************************************************/

#ifdef HAVE_MMCSD
static const struct sam_hsmci_info_s *sam_hsmci_info(int slotno)
{
   const struct sam_hsmci_info_s *info = NULL;

#ifdef CONFIG_SAMA5_HSMCI0
#ifdef CONFIG_SAMA5_HSMCI1
  if (slotno == 0)
#endif
    {
      info = &g_hsmci0_info;
    }
#ifdef CONFIG_SAMA5_HSMCI1
  else
#endif
#endif

#ifdef CONFIG_SAMA5_HSMCI1
    {
      info = &g_hsmci1_info;
    }
#endif

  return info;
}
#endif

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

#if defined(CONFIG_SAMA5_HSMCI0) || defined(CONFIG_SAMA5_HSMCI1)
int sam_hsmci_initialize(int slotno, int minor)
{
#ifdef HAVE_MMCSD
  const struct sam_hsmci_info_s *info;
  int ret;

  /* Get the HSMI description */

  info = sam_hsmci_info(slotno);
  if (!info)
    {
      fdbg("No info for slotno %d\n", slotno);
      return -EINVAL;
    }

  /* Initialize card-detect and write-protect PIOs */

   sam_configpio(info->pincfg);

  /* Mount the SDIO-based MMC/SD block driver */
  /* First, get an instance of the SDIO interface */

  *info->hsmci = sdio_initialize(slotno);
  if (!*info->hsmci)
    {
      fdbg("Failed to initialize SDIO slot %d\n",  slotno);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(minor, *info->hsmci);
  if (ret != OK)
    {
      fdbg("Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  /* Configure card detect interrupts */

  sam_pioirq(info->pincfg);
  (void)irq_attach(info->irq, info->handler);
  sam_pioirqenable(info->irq);

  /* Then inform the HSMCI driver if there is or is not a card in the slot. */

   sdio_mediachange(*info->hsmci, sam_cardinserted(slotno));
#endif

  return OK;
}

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_HSMCI0) || defined(CONFIG_SAMA5_HSMCI1)
bool sam_cardinserted(int slotno)
{
#ifdef HAVE_MMCSD
  const struct sam_hsmci_info_s *info;
  bool inserted;

  /* Get the HSMI description */

  info = sam_hsmci_info(slotno);
  if (!info)
    {
      fdbg("No info for slotno %d\n", slotno);
      return false;
    }

  /* Get the state of the PIO pin */

  inserted = sam_pioread(info->pincfg);
  fvdbg("Slot %d inserted: %s\n", slotno, inserted ? "NO" : "YES");
  return !inserted;

#else /* HAVE_MMCSD */

  return false;

#endif /* HAVE_MMCSD */
}
#endif /* CONFIG_SAMA5_HSMCIO ||  CONFIG_SAMA5_HSMCI1 */

/****************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_HSMCI0) || defined(CONFIG_SAMA5_HSMCI1)
bool sam_writeprotected(int slotno)
{
  /* There are no write protect pins */

  return false;
}
#endif

#endif /* CONFIG_SAMA5_HSMCI0 || CONFIG_SAMA5_HSMCI1 */
