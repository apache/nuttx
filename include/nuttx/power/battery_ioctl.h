/****************************************************************************
 * include/nuttx/power/battery_ioctl.h
 * NuttX Battery IOCTLs definition
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_POWER_BATTERY_IOCTL_H
#define __INCLUDE_NUTTX_POWER_BATTERY_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All battery-related IOCTL commands must be defined in this header file
 * in order to assure that every IOCTL command is unique and will not be
 * aliased.
 */

#define BATIOC_STATE         _BATIOC(0x0001)
#define BATIOC_HEALTH        _BATIOC(0x0002)
#define BATIOC_ONLINE        _BATIOC(0x0003)
#define BATIOC_VOLTAGE       _BATIOC(0x0004)
#define BATIOC_CURRENT       _BATIOC(0x0005)
#define BATIOC_INPUT_CURRENT _BATIOC(0x0006)
#define BATIOC_CAPACITY      _BATIOC(0x0007)
#define BATIOC_OPERATE       _BATIOC(0x0008)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct batio_operate_msg_s
{
  uint8_t operate_type; /* Really enum batio_operate_e */
  union
  {
    uint32_t u32;
    uint8_t  u8[8];
  };
};

#if defined(CONFIG_I2C_BQ2429X)
enum batio_operate_e
{
  BATIO_OPRTN_NOP = 0,
  BATIO_OPRTN_BOOST,
  BATIO_OPRTN_CHARGE,
  BATIO_OPRTN_EN_TERM,
  BATIO_OPRTN_HIZ,
  BATIO_OPRTN_SYSOFF,
  BATIO_OPRTN_SYSON,
  BATIO_OPRTN_RESET,
  BATIO_OPRTN_WDOG,
  BATIO_OPRTN_END
};
#endif

#endif /* __INCLUDE_NUTTX_POWER_BATTERY_IOCTL_H */
