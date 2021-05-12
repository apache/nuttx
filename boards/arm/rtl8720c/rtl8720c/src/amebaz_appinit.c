/****************************************************************************
 * boards/arm/ameba/amebaZ/src/amebaz_appinit.c
 *
 *   Copyright (C) 2019 Xiaomi InC. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <crc32.h>

#include <sys/boardctl.h>

#include "amebaz.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_BOARDCTL

static uint64_t board_did;
static uint8_t  board_key[16];

static int ameba_devinfo_init(void)
{
  extern int efuse_logical_read(uint16_t laddr,
                                uint16_t size, uint8_t *pbuf);
  extern uint32_t hal_user_otp_get(uint8_t *puser_otp);
  uint8_t content[6 + 32 + 1];
  uint32_t crc32_i = 0;
  uint32_t crc32_c;
  int ret;
  int i;

  /* | Mac (6 byte) | did (8 byte) | key (16 byte) | */

  ret = efuse_logical_read(0x11a, 6, content);
  if (ret < 0)
    {
      return ret;
    }

  hal_user_otp_get(content + 6);

  crc32_c = crc32part(content, 30, 0);
  for (i = 0; i < 4; i++)
    {
      crc32_i <<= 8;
      crc32_i |= content[i + 30];
    }

  if (crc32_i != crc32_c)
    {
      return -EINVAL;
    }

  for (i = 0; i < 8; i++)
    {
      board_did = 256 * board_did + content[i + 6];
    }

  memcpy(board_key, content + 6 + 8, sizeof(board_key));

  return 0;
}

#ifdef CONFIG_BOARDCTL_IOCTL

#define BOARDIOC_UNIQUEKEY       (BOARDIOC_USER + 1)

int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  switch (cmd)
    {
      case BOARDIOC_UNIQUEKEY:
        {
          memcpy((void *)arg, board_key, sizeof(board_key));
        }
        break;
      default:
        return -EINVAL;
    }

    return OK;
}
#endif /* CONFIG_BOARDCTL_IOCTL */

#ifdef CONFIG_BOARDCTL_UNIQUEID

int board_uniqueid(uint8_t *uniqueid)
{
  *(uint64_t *)uniqueid = board_did;
  return OK;
}

#endif /* CONFIG_BOARDCTL_UNIQUEID */

int board_app_initialize(uintptr_t arg)
{
  extern uint32_t get_cur_fw_idx(void);
  char devpath[32];

#ifdef CONFIG_WATCHDOG
  ameba_wdt_initialize();
#endif

#ifndef CONFIG_BOARD_LATE_INITIALIZE
  amebaz_bringup();
#endif

  ameba_flash_init();

  ameba_devinfo_init();

  snprintf(devpath, sizeof(devpath), "/dev/fw%d",
           (get_cur_fw_idx() == 1) ?  2 : 1);
  setenv("SYS_UPDATE_FIRMWARE_PATH", devpath, 1);

  snprintf(devpath, sizeof(devpath), "/dev/fw%d",
           get_cur_fw_idx());
  setenv("SYS_RUNNING_FIRMWARE_PATH", devpath, 1);

  return 0;
}

#ifdef CONFIG_BOARDCTL_RESET
int board_reset(int status)
{
  ameba_reset(status);

  return 0;
}
#endif /* CONFIG_BOARDCTL_RESET */

#endif /* CONFIG_LIB_BOARDCTL */
