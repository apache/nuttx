/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_sdio.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <arch/board/board.h>

#include "mpfs_coremmc.h"
#include "mpfs_emmcsd.h"
#include "mpfs_sdio_dev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno)
{
  switch (slotno)
    {
    case 0:
#ifdef CONFIG_MPFS_EMMCSD
      return mpfs_emmcsd_sdio_initialize(slotno);
#endif

    case 1:
#ifdef CONFIG_MPFS_COREMMC
      return mpfs_coremmc_sdio_initialize(slotno);
#endif

    default:
      break;
    }

  mcerr("sdio slot number %d not supported!\n", slotno);
  return NULL;
}

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possible from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;

  if (!dev)
    {
      mcerr("sdio device not found\n");
      return;
    }

  switch (priv->hw_base)
    {
#ifdef CONFIG_MPFS_EMMCSD
    case MPFS_EMMC_SD_BASE:
      mpfs_emmcsd_sdio_mediachange(dev, cardinslot);
      return;
#endif

#ifdef CONFIG_MPFS_COREMMC
    case CONFIG_MPFS_COREMMC_BASE:
      mpfs_coremmc_sdio_mediachange(dev, cardinslot);
      return;
#endif

    default:
      break;
    }

  mcerr("Invalid sdio base address\n");
}

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
{
  struct mpfs_dev_s *priv = (struct mpfs_dev_s *)dev;

  if (!dev)
    {
      mcerr("sdio device not found\n");
      return;
    }

  switch (priv->hw_base)
    {
#ifdef CONFIG_MPFS_EMMCSD
    case MPFS_EMMC_SD_BASE:
      mpfs_emmcsd_sdio_wrprotect(dev, wrprotect);
      return;
#endif

#ifdef CONFIG_MPFS_COREMMC
    case CONFIG_MPFS_COREMMC_BASE:
      mpfs_coremmc_sdio_wrprotect(dev, wrprotect);
      return;
#endif

    default:
      mcerr("Invalid sdio base address\n");
      break;
    }
}
