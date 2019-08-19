/****************************************************************************
 * boards/risc-v/gap8/gapduino/src/gapduino_sysinit.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
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

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include "gap8.h"
#include "gap8_fll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Used to communicate with plpbridge */

struct _debug_struct
{
  /* Used by external debug bridge to get exit status when using the board */

  uint32_t exitStatus;

  /* Printf */

  uint32_t useInternalPrintf;
  uint32_t putcharPending;
  uint32_t putcharCurrent;
  uint8_t putcharBuffer[128];

  /* Debug step, used for showing progress to host loader */

  uint32_t debugStep;
  uint32_t debugStepPending;

  /* Requests */

  uint32_t firstReq;
  uint32_t lastReq;
  uint32_t firstBridgeReq;

  uint32_t notifReqAddr;
  uint32_t notifReqValue;

  uint32_t bridgeConnected;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Place a dummy debug struct */

struct _debug_struct Debug_Struct =
{
  .useInternalPrintf = 1,
};

uint32_t g_current_freq = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gapuino_sysinit
 *
 * Description:
 *   Initialize cache, clock, etc.
 *
 ****************************************************************************/

void gapuino_sysinit(void)
{
  SCBC->ICACHE_ENABLE = 0xffffffff;
  gap8_setfreq(CONFIG_CORE_CLOCK_FREQ);

  /* For debug usage */

  g_current_freq = gap8_getfreq();
}
