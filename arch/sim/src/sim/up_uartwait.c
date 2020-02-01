/****************************************************************************
 * arch/sim/src/sim/up_uartwait.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/semaphore.h>

#include "up_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_uartavail;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: simuart_initialize
 ****************************************************************************/

void simuart_initialize(void)
{
  /* The g_uartavail semaphore is used for signaling and, hence, should not
   * have priority inheritance enabled.
   */

  nxsem_init(&g_uartavail, 0, 0);
  nxsem_setprotocol(&g_uartavail, SEM_PRIO_NONE);
}

/****************************************************************************
 * Name: simuart_post
 ****************************************************************************/

void simuart_post(void)
{
  nxsem_post(&g_uartavail);
}

/****************************************************************************
 * Name: simuart_wait
 ****************************************************************************/

void simuart_wait(void)
{
  /* Should only fail if interrupted by a signal */

  while (nxsem_wait(&g_uartavail) < 0);
}
