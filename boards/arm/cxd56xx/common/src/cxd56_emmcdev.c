/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_emmcdev.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>
#include <debug.h>
#include <sys/mount.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include "cxd56_emmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SFC_DEVNO
#  define CONFIG_SFC_DEVNO 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_emmc_initialize
 *
 * Description:
 *   Initialize the eMMC device and mount the file system.
 *
 ****************************************************************************/

int board_emmc_initialize(void)
{
  int ret;

  /* Power on the eMMC device */

  ret = board_power_control(POWER_EMMC, true);
  if (ret)
    {
      ferr("ERROR: Failed to power on eMMC. %d\n", ret);
      return -ENODEV;
    }

  /* Initialize the eMMC deivce */

  ret = cxd56_emmcinitialize();
  if (ret < 0)
    {
      ferr("ERROR: Failed to initialize eMMC. %d\n ", ret);
      return -ENODEV;
    }

  /* Mount the eMMC deivce */

  ret = mount("/dev/emmc0", "/mnt/emmc", "vfat", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the eMMC. %d\n", errno);
    }

  return ret;
}
