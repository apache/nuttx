/****************************************************************************
 * drivers/timers/mcp7941x.h
 *
 *   Copyright (C) 2019 Abdelatif Guettouche. All rights reserved.
 *   Author: 2019 Abdelatif Guettouche <abdelatif.guettouche@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#ifndef __DRIVERS_TIMERS_MCP7941X_H
#define __DRIVERS_TIMERS_MCP7941X_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCP7941X_REG_TIME_SEC         0x00  /* Seconds register. */
#  define MCP7941X_TIME_SEC_10SEC     (7 << 4)
#  define MCP7941X_TIME_SEC_ST        (1 << 7)
#  define MCP7941X_TIME_SEC_BCDMASK   0x7F

#define MCP7941X_REG_TIME_MIN         0x01  /* Minutes register. */
#  define MCP7941X_TIME_MIN_10MIN     (7 << 4)
#  define MCP7941X_TIME_MIN_BCDMASK   0x7F

#define MCP7941X_REG_TIME_HOUR        0x02  /* Hours register. */
#  define MCP7941X_TIME_HOUR_10HOUR   (3 << 4)
#  define MCP7941X_TIME_HOUR_AMPM     (1 << 5)
#  define MCP7941X_TIME_HOUR_1224     (1 << 6)
#  define MCP7941X_TIME_HOUR_BCDMASK  0x3F

#define MCP7941X_REG_TIME_DAY         0x03  /* Day register. */
#  define MCP7941X_TIME_DAY_VBATEN    (1 << 3)
#  define MCP7941X_TIME_DAY_VBAT      (1 << 4)
#  define MCP7941X_TIME_DAY_OSCON     (1 << 5)
#  define MCP7941X_TIME_DAY_BCDMASK   0x07

#define MCP7941X_REG_TIME_DATE        0x04  /* Date register. */
#  define MCP7941X_TIME_DATE_10DATE   (3 << 4)
#  define MCP7941X_TIME_DATE_BCDMASK  0x3F

#define MCP7941X_REG_TIME_MONTH       0x05  /* Month register. */
#  define MCP7941X_TIME_MONTH_10MONTH (1 << 4)
#  define MCP7941X_TIME_MONTH_LP      (1 << 5)
#  define MCP7941X_TIME_MONTH_BCDMASK 0x1F

#define MCP7941X_REG_TIME_YEAR        0x06  /* Year register. */
#  define MCP7941X_TIME_YEAR_10YEAR   (15 << 4)
#  define MCP7941X_TIME_YEAR_BCDMASK  0xFF

#define MCP7941X_REG_CTRL             0x07  /* Control register. */
#  define MCP7941X_CTRL_RS0           (1 << 0)
#  define MCP7941X_CTRL_RS1           (1 << 1)
#  define MCP7941X_CTRL_RS2           (1 << 2)
#  define MCP7941X_CTRL_EXTOSC        (1 << 3)
#  define MCP7941X_CTRL_ALM0          (1 << 4)
#  define MCP7941X_CTRL_ALM1          (1 << 5)
#  define MCP7941X_CTRL_SQWE          (1 << 6)
#  define MCP7941X_CTRL_OUT           (1 << 7)

#define MCP7941X_REG_CALIB            0x08  /* Calibration register. */
#define MCP7941X_REG_ID               0x09  /* ID register. */

#endif /* __DRIVERS_TIMERS_MCP7941X_H */
