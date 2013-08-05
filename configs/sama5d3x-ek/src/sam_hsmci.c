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


/* SPI Chip Selects *****************************************************************/
/* Both the Ronetix and Embest versions of the SAMAD3x CPU modules include an
 * Atmel AT25DF321A, 32-megabit, 2.7-volt SPI serial flash.  The SPI
 * connection is as follows:
 *
 *   AT25DF321A      SAMA5
 *   --------------- -----------------------------------------------
 *   SI              PD11 SPI0_MOSI
 *   SO              PD10 SPI0_MIS0
 *   SCK             PD12 SPI0_SPCK
 *   /CS             PD13 via NL17SZ126 if JP1 is closed (See below)
 *
 * JP1 and JP2 seem to related to /CS on the Ronetix board, but the usage is
 * less clear.  For the Embest module, JP1 must be closed to connect /CS to
 * PD13; on the Ronetix schematic, JP11 seems only to bypass a resistor (may
 * not be populated?).  I think closing JP1 is correct in either case.
 */

#define GPIO_AT25_NPCS0 (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                         GPIO_PORT_PIOD | GPIO_PIN13)
#define AT25_PORT       SPI0_CS0


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

#include "sam_hsmci.h"
#include "sama5d3x-ek.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* This needs to be extended.  The card detect GPIO must be configured as an
 * interrupt.  When the interrupt indicating that a card has been inserted
 * or removed is received, this function must call sio_mediachange() to
 * handle that event.
 */

#warning "Card detect interrupt handling needed"

/* Configuration ************************************************************/

#define HAVE_MMCSD  1

/* Can't support MMC/SD if the card interface(s) are not enable */

#if !defined(CONFIG_SAMA5_HSMCI0) && !defined(CONFIG_SAMA5_HSMCI0)
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_MMCSD
#endif

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
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: sam_hsmci_gpioinit
 *
 * Description:
 *   Initialize HSMCI support.  This function is called very early in board
 *   initialization.
 *
 ************************************************************************************/

#ifdef HAVE_MMCSD
static void sam_hsmci_gpioinit(int slotno)
{
#ifdef CONFIG_SAMA5_HSMCI0
#ifdef CONFIG_SAMA5_HSMCI1
  if (slotno == 0)
#endif
    {
      sam_configgpio(GPIO_MCI0_CD);
    }
#ifdef CONFIG_SAMA5_HSMCI1
  else
#endif
#endif

#ifdef CONFIG_SAMA5_HSMCI1
    {
      sam_configgpio(GPIO_MCI1_CD);
    }
#endif
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
  FAR struct sdio_dev_s *sdio;
  int ret;

  /* Initialize card-detect and write-protect GPIOs */

  sam_hsmci_gpioinit(slotno);

  /* Mount the SDIO-based MMC/SD block driver */
  /* First, get an instance of the SDIO interface */

  sdio = sdio_initialize(slotno);
  if (!sdio)
    {
      fdbg("Failed to initialize SDIO slot %d\n",  slotno);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(minor, sdio);
  if (ret != OK)
    {
      fdbg("Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  /* Then inform the HSMCI driver if there is or is not a card in the slot. */

   sdio_mediachange(sdio, sam_cardinserted(slotno));
#endif

  return OK;
}

/************************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ************************************************************************************/

#if defined(CONFIG_SAMA5_HSMCI0) || defined(CONFIG_SAMA5_HSMCI1)
bool sam_cardinserted(int slotno)
{
#ifdef HAVE_MMCSD

#ifdef CONFIG_SAMA5_HSMCI0
#ifdef CONFIG_SAMA5_HSMCI1
  if (slotno == 0)
#endif /* CONFIG_SAMA5_HSMCI1 */
    {
      bool inserted = sam_gpioread(GPIO_MCI0_CD);
      fvdbg("Slot 0 inserted: %s\n", inserted ? "NO" : "YES");
      return !inserted;
    }

#ifdef CONFIG_SAMA5_HSMCI1
  else
#endif /* CONFIG_SAMA5_HSMCI1 */
#endif /* CONFIG_SAMA5_HSMCI0 */

#ifdef CONFIG_SAMA5_HSMCI1
    {
      bool inserted = sam_gpioread(GPIO_MCI1_CD);
      fvdbg("Slot 1 inserted: %s\n", inserted ? "NO" : "YES");
      return !inserted;
    }

#endif /* CONFIG_SAMA5_HSMCI1 */
#else /* HAVE_MMCSD */

  return false;

#endif /* HAVE_MMCSD */
}
#endif /* CONFIG_SAMA5_HSMCIO ||  CONFIG_SAMA5_HSMCI1 */

/************************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ************************************************************************************/

#if defined(CONFIG_SAMA5_HSMCI0) || defined(CONFIG_SAMA5_HSMCI1)
bool sam_writeprotected(int slotno)
{
  /* There are no write protect pins */

  return false;
}
#endif

#endif /* CONFIG_SAMA5_HSMCI0 || CONFIG_SAMA5_HSMCI1 */
