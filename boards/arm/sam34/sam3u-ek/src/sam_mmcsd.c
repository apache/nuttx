/****************************************************************************
 * boards/arm/sam34/sam3u-ek/src/sam_mmcsd.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <debug.h>

#include "sam_gpio.h"
#include "sam3u-ek.h"

#ifdef CONFIG_SAM34_HSMCI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This needs to be extended.
 * The card detect GPIO must be configured as an interrupt.
 * when the interrupt indicating that a card has been inserted or removed
 * is received, this function must call sio_mediachange() to handle that
 * event.
 * See arch/arm/src/sam34/sam_hsmci.h for more information.
 *
 * Also see the SAMA5D3x-EK implementation of this same logic.
 * The card detect interrupt handling should be a drop-in.
 */

#ifdef GPIO_MCI_CD
#  warning "Card detect interrupt handling needed"
#endif

/* Usually defined in NuttX header files */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_hsmciinit
 *
 * Description:
 *   Initialize HSMCI support.  This function is called very early in board
 *   initialization.
 *
 ****************************************************************************/

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

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

bool sam_cardinserted(unsigned char slot)
{
  if (slot == 0)
    {
#ifdef GPIO_MCI_CD
      bool inserted = sam_gpioread(GPIO_MCI_CD);
      finfo("inserted: %s\n", inserted ? "NO" : "YES");
      return !inserted;
#else
      return true;
#endif
    }

  return false;
}

/****************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

bool sam_writeprotected(unsigned char slot)
{
  if (slot == 0)
    {
#ifdef GPIO_MCI_WP
      bool protected = sam_gpioread(GPIO_MCI_WP);
      finfo("protected: %s\n", inserted ? "YES" : "NO");
      return protected;
#else
      return false;
#endif
    }

  return false;
}

#endif /* CONFIG_SAM34_HSMCI */
