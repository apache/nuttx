/************************************************************************************
 * configs/sam3u-ek/src/sam_mmcsd.c
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <debug.h>

#include "sam_gpio.h"
#include "sam3u-ek.h"

#ifdef CONFIG_SAM34_HSMCI

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* This needs to be extended.  The card detect GPIO must be configured as an interrupt.
 * when the interrupt indicating that a card has been inserted or removed is received,
 * this function must call sio_mediachange() to handle that event.  See
 * arch/arm/src/sam34/sam_hsmci.h for more information.
 *
 * Also see the SAMA5D3x-EK implementation of this same logic.  The card detect
 * interrupt handling should be a drop-in.
 */

#ifdef GPIO_MCI_CD
#  warning "Card detect interrupt handling needed"
#endif

/* Usually defined in NuttX header files */

#ifndef OK
#  define OK 0
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_hsmciinit
 *
 * Description:
 *   Initialize HSMCI support.  This function is called very early in board
 *   initialization.
 *
 ************************************************************************************/

int sam_hsmciinit(void)
{
#ifdef GPIO_MCI_CD
  sam_configgpio(GPIO_MCI_CD);
#endif
#ifdef GPIO_MCI_WP
  sam_configgpio(GPIO_MCI_WP);
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

bool sam_cardinserted(unsigned char slot)
{
  if (slot == 0)
    {
#ifdef GPIO_MCI_CD
      bool inserted = sam_gpioread(GPIO_MCI_CD);
      fvdbg("inserted: %s\n", inserted ? "NO" : "YES");
      return !inserted;
#else
      return true;
#endif
    }
  return false;
}

/************************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ************************************************************************************/

bool sam_writeprotected(unsigned char slot)
{
  if (slot == 0)
    {
#ifdef GPIO_MCI_WP
      bool protected = sam_gpioread(GPIO_MCI_WP);
      fvdbg("protected: %s\n", inserted ? "YES" : "NO");
      return protected;
#else
      return false;
#endif
    }
  return false;
}

#endif /* CONFIG_SAM34_HSMCI */
