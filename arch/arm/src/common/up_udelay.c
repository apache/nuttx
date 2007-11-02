/****************************************************************************
 * common/up_udelay.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
#include <nuttx/arch.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define CONFIG_BOARD_LOOPSPERUSEC   ((CONFIG_BOARD_LOOPSPERMSEC+500)/1000)
#define CONFIG_BOARD_LOOPSPER10USEC ((CONFIG_BOARD_LOOPSPERMSEC+50)/100)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Delay inline for the requested number of microseconds.  NOTE:  Because
 *   of all of the setup, several microseconds will be lost before the actual
 *   timing looop begins.  Thus, the delay will always be a few microseconds
 *   longer than requested.
 *
 *   *** NOT multi-tasking friendly ***
 *
 * ASSUMPTIONS:
 *   The setting CONFIG_BOARD_LOOPSPERMSEC has been calibrated
 *
 ****************************************************************************/

void up_udelay(unsigned int microseconds)
{
  volatile int    i;
  volatile int    j;
  register uint32 loops;

  /* The value of microseconds should be less than 1000.  If not, then we
   * will perform millescond delays until it is.
   */

  while (microseconds > 1000)
    {
      for (j = 0; j < CONFIG_BOARD_LOOPSPERMSEC; j++)
        {
        }
      microseconds -= 1000;
    }

  /* The numerator of the 'loops' below will overflow if CONFIG_BOARD_LOOPSPERMSEC
   * is larger than (4*1024*1024*1024 - 500)/999 = 4,299,266.06
   */

#if CONFIG_BOARD_LOOPSPERMSEC >= 4299266
  while (microseconds > 500)
    {
      for (j = 0; j < ((CONFIG_BOARD_LOOPSPERMSEC+1)/2); j++)
        {
        }
      microseconds -= 500;
    }
#endif

  /* The overflow could still occur if CONFIG_BOARD_LOOPSPERMSEC is larger than
   * (4*1024*1024*1024 - 500)/499 = 8,607,147.89
   */

#if CONFIG_BOARD_LOOPSPERMSEC >= 8607147
# warning "Overflow in loops calculation is possible"
#endif

  /* Caculate the number of loops need to produce the required usec delay */

  loops = (CONFIG_BOARD_LOOPSPERMSEC * microseconds + 500) / 1000;
  for (j = 0; j < loops; j++)
    {
    }
}
