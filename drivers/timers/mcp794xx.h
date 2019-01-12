/****************************************************************************
 * drivers/timers/mcp794xx.h
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

#ifndef __DRIVERS_TIMERS_MCP794XX_H
#define __DRIVERS_TIMERS_MCP794XX_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCP794XX_REG_RTCSEC          0x00  /* Seconds register. */
#  define MCP794XX_RTCSEC_10SEC      (7 << 4)
#  define MCP794XX_RTCSEC_ST         (1 << 7)
#  define MCP794XX_RTCSEC_BCDMASK    0x7F

#define MCP794XX_REG_RTCMIN          0x01  /* Minutes register. */
#  define MCP794XX_RTCMIN_10MIN      (7 << 4)
#  define MCP794XX_RTCMIN_BCDMASK    0x7F

#define MCP794XX_REG_RTCHOUR         0x02  /* Hours register. */
#  define MCP794XX_RTCHOUR_10HOUR    (3 << 4)
#  define MCP794XX_RTCHOUR_AMPM      (1 << 5)
#  define MCP794XX_RTCHOUR_1224      (1 << 6)
#  define MCP794XX_RTCHOUR_BCDMASK   0x3F

#define MCP794XX_REG_RTCWKDAY        0x03  /* Day register. */
#  define MCP794XX_RTCWKDAY_VBATEN   (1 << 3)
#  define MCP794XX_RTCWKDAY_PWRFAIL  (1 << 4)
#  define MCP794XX_RTCWKDAY_OSCRUN   (1 << 5)
#  define MCP794XX_RTCWKDAY_BCDMASK  0x07

#define MCP794XX_REG_RTCDATE         0x04  /* Date register. */
#  define MCP794XX_RTCDATE_10DATE    (3 << 4)
#  define MCP794XX_RTCDATE_BCDMASK   0x3F

#define MCP794XX_REG_RTCMTH          0x05  /* Month register. */
#  define MCP794XX_RTCMTH_10MONTH    (1 << 4)
#  define MCP794XX_RTCMTH_LPYR       (1 << 5)
#  define MCP794XX_RTCMTH_BCDMASK    0x1F

#define MCP794XX_REG_RTCYEAR         0x06  /* Year register. */
#  define MCP794XX_RTCYEAR_10YEAR    (15 << 4)
#  define MCP794XX_RTCYEAR_BCDMASK   0xFF

#define MCP794XX_REG_CONTROL         0x07  /* Control register. */
#  define MCP794XX_CONTROL_SQWFS0    (1 << 0)
#  define MCP794XX_CONTROL_SQWFS1    (1 << 1)
#  define MCP794XX_CONTROL_CRSTRIM   (1 << 2)
#  define MCP794XX_CONTROL_EXTOSC    (1 << 3)
#  define MCP794XX_CONTROL_ALM0EN    (1 << 4)
#  define MCP794XX_CONTROL_ALM1EN    (1 << 5)
#  define MCP794XX_CONTROL_SQWEN     (1 << 6)
#  define MCP794XX_CONTROL_OUT       (1 << 7)

#define MCP794XX_REG_OSCTRIM         0x08  /* Calibration register. */
#  define MCP794XX_OSCTRIM_SIGN      (1 << 7)

#endif /* __DRIVERS_TIMERS_MCP794XX_H */
