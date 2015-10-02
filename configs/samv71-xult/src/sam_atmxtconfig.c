/************************************************************************************
 * configs/samv71-xult/src/sam_atmxtconfig.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include "atmxt-xpro.h"

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifdef HAVE_MAXTOUCH
/* Configuration arrays referenced in g_atmxt_config[] */

static const uint8_t g_id26[0x08] =
{
  0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t g_id07[0x04] =
{
  0x20, 0x10, 0x4b, 0x82
};

static const uint8_t g_id08[0x0a] =
{
  0x0f, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00, 0x00,
  0x32, 0x19
};

static const uint8_t g_id09[0x24] =
{
  0x8b, 0x00, 0x00, 0x0e, 0x08, 0x00, 0x80, 0x32,
  0x05, 0x02, 0x0a, 0x03, 0x03, 0x20, 0x04, 0x0f,
  0x0f, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x20, 0x30,
  0x64, 0x54, 0x9e, 0x30, 0xd4, 0x50, 0x00, 0x0a,
  0x00, 0x00, 0x02, 0x02
};

static const uint8_t g_id0f[0x0b] =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00
};

static const uint8_t g_id12[0x02] =
{
  0x00, 0x00
};

static const uint8_t g_id19[0x0f] =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t g_id28[0x05] =
{
  0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t g_id2a[0x0a] =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00
};

static const uint8_t g_id2e[0x09] =
{
  0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03, 0x00,
  0x00
};

static const uint8_t g_id37[0x06] =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t g_id38[0x21] =
{
  0x02, 0x00, 0x01, 0x18, 0x1e, 0x1e, 0x1e, 0x1e,
  0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e,
  0x1e, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00
};

static const uint8_t g_id3e[0x36] =
{
  0x01, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x05, 0x0a, 0x0f, 0x16, 0x23, 0x05, 0x00,
  0x0a, 0x05, 0x05, 0x80, 0x19, 0x19, 0x34, 0x0c,
  0x64, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t g_id41[0x14] =
{
  0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

static const uint8_t g_id42[0x03] =
{
  0x02, 0x14, 0x0f
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* List of configuration settings.  Terminated with an entry with nbytes == 0 and
 * bytes == NULL;
 */

const struct atmxt_config_s g_atmxt_config[] =
{
  {
    .addr   = 0x0110,
    .id     = 0x26,
    .nbytes = 0x08,
    .bytes  = g_id26
  },
  {
    .addr   = 0x0118,
    .id     = 0x07,
    .nbytes = 0x04,
    .bytes  = g_id07
  },
  {
    .addr   = 0x011c,
    .id     = 0x08,
    .nbytes = 0x0a,
    .bytes  = g_id08
  },
  {
    .addr   = 0x0126,
    .id     = 0x09,
    .nbytes = 0x24,
    .bytes  = g_id09
  },
  {
    .addr   = 0x014a,
    .id     = 0x0f,
    .nbytes = 0x0b,
    .bytes  = g_id0f
  },
  {
    .addr   = 0x0155,
    .id     = 0x12,
    .nbytes = 0x02,
    .bytes  = g_id12
  },
  {
    .addr   = 0x0157,
    .id     = 0x19,
    .nbytes = 0x0f,
    .bytes  = g_id19
  },
  {
    .addr   = 0x0166,
    .id     = 0x28,
    .nbytes = 0x05,
    .bytes  = g_id28
  },
  {
    .addr   = 0x016b,
    .id     = 0x2a,
    .nbytes = 0x0a,
    .bytes  = g_id2a
  },
  {
    .addr   = 0x0175,
    .id     = 0x2e,
    .nbytes = 0x09,
    .bytes  = g_id2e
  },
  {
    .addr   = 0x017e,
    .id     = 0x37,
    .nbytes = 0x06,
    .bytes  = g_id37
  },
  {
    .addr   = 0x0184,
    .id     = 0x38,
    .nbytes = 0x21,
    .bytes  = g_id38
  },
  {
    .addr   = 0x01a5,
    .id     = 0x3e,
    .nbytes = 0x36,
    .bytes  = g_id3e
  },
  {
    .addr   = 0x01db,
    .id     = 0x41,
    .nbytes = 0x14,
    .bytes  = g_id41
  },
  {
    .addr   = 0x01ef,
    .id     = 0x42,
    .nbytes = 0x03,
    .bytes  = g_id42
  },
  {
    .addr   = 0x0000,
    .id     = 0x00,
    .nbytes = 0x00,
    .bytes  = (FAR uint8_t *)0
  }
};

#endif

