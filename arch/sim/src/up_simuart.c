/****************************************************************************
 * arch/sim/src/up_simuart.c
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

#include <unistd.h>
#include <termios.h>
#include <pthread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Simulated console UART input buffer size */
/* Must match the defintion in up_internal.h */

#define SIMUART_BUFSIZE 256

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_uartbuffer[SIMUART_BUFSIZE];
static volatile int  g_uarthead;
static volatile int  g_uarttail;

/****************************************************************************
 * NuttX Domain Public Function Prototypes
 ****************************************************************************/

void sched_lock(void);
void sched_unlock(void);

void simuart_initialize(void);
void simuart_post(void);
void simuart_wait(void);

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
 * Name: simuart_thread
 ****************************************************************************/

static void *simuart_thread(void *arg)
{
  unsigned char ch;
  ssize_t nread;
  int next;
  int prev;

  /* Now loop, collecting a buffering data from stdin forever */

  for (;;)
    {
      /* Read one character from stdin */

      nread = read(0, &ch, 1);

      /* Check for failures (but don't do anything) */

      if (nread == 1)
        {
          sched_lock();

          /* Get the index to the next slot in the UART buffer */

          prev = g_uarthead;
          next = prev + 1;
          if (next >= SIMUART_BUFSIZE)
            {
              next = 0;
            }

          /* Would adding this character cause an overflow? */

          if (next != g_uarttail)
            {
              /* No.. Add the character to the UART buffer */

              g_uartbuffer[prev] = ch;

              /* Update the head index (BEFORE posting) */

              g_uarthead = next;

              /* Was the buffer previously empty? */

              if (prev == g_uarttail)
                {
                  /* Yes.. signal any (NuttX) threads waiting for serial
                   * input.
                   */

                  simuart_post();
                }
            }

          sched_unlock();
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: simuart_putraw
 ****************************************************************************/

int simuart_putraw(int ch)
{
  ssize_t nwritten;

  nwritten = write(1, &ch, 1);
  if (nwritten != 1)
    {
      return -1;
    }

  return ch;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: simuart_start
 ****************************************************************************/

void simuart_start(void)
{
  pthread_t tid;

  /* This thread runs in the host domain */
  /* Perform the NuttX domain initialization */

  simuart_initialize();

  /* Put stdin into raw mode */

  setrawmode();

  /* Start the simulated UART thread -- all default settings; no error
   * checking.
   */

  (void)pthread_create(&tid, NULL, simuart_thread, NULL);
}

/****************************************************************************
 * Name: simuart_putc
 ****************************************************************************/

int simuart_putc(int ch)
{
  int ret = ch;

  if (ch == '\n')
    {
      ret = simuart_putraw('\r');
    }

  if (ret >= 0)
    {
      ret = simuart_putraw(ch);
    }

  return ret;
}

/****************************************************************************
 * Name: simuart_getc
 ****************************************************************************/

int simuart_getc(void)
{
  int index;
  int ch;

  for (;;)
    {
      /* Wait for a byte to become available */

      simuart_wait();

      /* The UART buffer show be non-empty... check anyway */

      if (g_uarthead != g_uarttail)
        {
          /* Take the next byte from the tail of the buffer */

          index = g_uarttail;
          ch    = (int)g_uartbuffer[index];

          /* Increment the tai index (with wrapping) */

          if (++index >= SIMUART_BUFSIZE)
            {
              index = 0;
            }

          g_uarttail = index;
          return ch;
        }
    }
}
