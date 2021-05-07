/****************************************************************************
 * drivers/timers/mcp794xx.h
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
