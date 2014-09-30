/****************************************************************************
 * arch/sim/src/up_hostusleep.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <termios.h>

#include "sim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

char g_uartbuffer[CONFIG_SIM_UART_BUFSIZE];
volatile int  g_uarthead;
volatile int  g_uarttail;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setrawmode
 ****************************************************************************/

static void setrawmode(void)
{
  struct termios term;

  (void)tcgetattr(0, &term);

  term.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  term.c_oflag &= ~OPOST;
  term.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  term.c_cflag &= ~(CSIZE | PARENB);
  term.c_cflag |= CS8;

  (void)tcsetattr(0, TCSANOW, &term);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_simuart
 ****************************************************************************/

void *up_simuart(void *arg)
{
  unsigned char ch;
  ssize_t nread;

  /* This thread runs in the host domain */
  /* Initialize the NuttX domain semaphore */

  uart_wait_initialize();

  /* Put stdin into raw mode */

  setrawmode();

  /* Now loop, collecting a buffering data from stdin forever */

  for (;;)
    {
      /* Read one character from stdin */

      nread = read(0, ch, 1);

      /* Check for failures (but don't do anything) */

      if (nread == 1)
        {
          /* Get the index to the next slot in the UART buffer */

          int next = g_uarthead + 1;
          if (next >= CONFIG_SIM_UART_BUFSIZE)
            {
              next = 0;
            }

          /* Would adding this character cause an overflow? */

          if (next != g_uarttail)
            {
              /* No.. Add the character to the UART buffer */

              g_uartbuffer[g_uarthead] = ch;

              /* Was the buffer previously empty? */

              if (g_uarthead == g_uarttail)
                {
                  /* Yes.. signal any (NuttX) threads waiting for serial
                   * input.
                   */

                  up_simuart_post();
                }

              /* Update the head index */

              g_uarthead = next;
            }
        }
    }

  return NULL;
}
