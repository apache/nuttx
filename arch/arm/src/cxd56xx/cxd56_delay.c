/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_delay.c
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
#include <sys/types.h>
#include <nuttx/arch.h>

#include <stdint.h>

#include "cxd56_clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD56XX_LOOPSPERMSEC_156MHZ 7428ull
#define CXD56XX_LOOPSPERMSEC_BY_CLOCK(clock) \
  (uint32_t)(CXD56XX_LOOPSPERMSEC_156MHZ * (clock) / 156000000ull)

/* Adjust manually to be up_udelay(1000) is neary equal with up_udelay(999) */

#define CXD56XX_LOOPSPERUSEC_ADJUST 810ul;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_mdelay
 *
 * Description:
 *   Delay inline for the requested number of milliseconds.
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void up_mdelay(unsigned int milliseconds)
{
  volatile unsigned int i;
  volatile unsigned int j;
  uint32_t clock = cxd56_get_cpu_baseclk();
  uint32_t loops = CXD56XX_LOOPSPERMSEC_BY_CLOCK(clock);

  for (i = 0; i < milliseconds; i++)
    {
      for (j = 0; j < loops; j++)
        {
        }
    }
}

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Delay inline for the requested number of microseconds.  NOTE:  Because
 *   of all of the setup, several microseconds will be lost before the actual
 *   timing loop begins.  Thus, the delay will always be a few microseconds
 *   longer than requested.
 *
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void up_udelay(useconds_t microseconds)
{
  volatile unsigned int i;
  volatile unsigned int j;
  uint32_t clock = cxd56_get_cpu_baseclk();
  uint32_t loops = CXD56XX_LOOPSPERMSEC_BY_CLOCK(clock);
  uint32_t milliseconds = microseconds / 1000;

  for (i = 0; i < milliseconds; i++)
    {
      for (j = 0; j < loops; j++)
        {
        }
    }

  loops = loops * (microseconds % 1000) / CXD56XX_LOOPSPERUSEC_ADJUST;
  for (i = 0; i < loops; i++)
    {
    }
}
