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

#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Simulated console UART input buffer size */
/* Must match the defintion in up_internal.h */

#define SIMUART_BUFSIZE 256

/* The design for how we signal UART data availability is up in the air */

#undef CONFIG_SIM_UART_DATAPOST

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_uartbuffer[SIMUART_BUFSIZE];
static volatile int  g_uarthead;
static volatile int  g_uarttail;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef CONFIG_SIM_UART_DATAPOST
volatile int g_uart_data_available;
#endif

/****************************************************************************
 * NuttX Domain Public Function Prototypes
 ****************************************************************************/

void sched_lock(void);
void sched_unlock(void);

void simuart_initialize(void);
void simuart_post(void);
void simuart_wait(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct termios g_cooked;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setrawmode
 ****************************************************************************/

static void setrawmode(void)
{
  struct termios raw;

  /* Get the current stdin terminal mode */

  (void)tcgetattr(0, &g_cooked);

  /* Switch to raw mode */

  memcpy(&raw, &g_cooked, sizeof(struct termios));

  raw.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  raw.c_oflag &= ~OPOST;
  raw.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  raw.c_cflag &= ~(CSIZE | PARENB);
  raw.c_cflag |= CS8;

  (void)tcsetattr(0, TCSANOW, &raw);
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

  for (; ; )
    {
      /* Read one character from stdin */

      nread = read(0, &ch, 1);

      /* Check for failures (but don't do anything) */

      if (nread == 1)
        {
#ifdef CONFIG_SIM_UART_DATAPOST
          sched_lock();
#endif
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

#ifdef CONFIG_SIM_UART_DATAPOST
                  simuart_post();
#else
                  g_uart_data_available = 1;
#endif
                }
            }

#ifdef CONFIG_SIM_UART_DATAPOST
          /* REVISIT:  This is very weird and scary here.  When sched_unlock()
           * is called, we may do a lonjmp() style context switch meaning
           * that the logic will be run running on this thread! (but with a
           * different stack).  So we do not get back here until the task
           * sleeps again.  I can't help but believe that that is going to
           * be a problem someday.
           */

          sched_unlock();
#endif
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

int simuart_getc(bool block)
{
  int index;
  int ch;

  /* Locking the scheduler should eliminate the race conditions in the
   * unlikely case of multiple reading threads.
   */

  sched_lock();
  for (; ; )
    {
      /* Wait for a byte to become available */

      if (!block && (g_uarthead == g_uarttail))
        {
          sched_unlock();
          return -EAGAIN;
        }

      while (g_uarthead == g_uarttail)
        {
          simuart_wait();
        }

      /* The UART buffer is non-empty...  Take the next byte from the tail
       * of the buffer.
       */

      index = g_uarttail;
      ch    = (int)g_uartbuffer[index];

      /* Increment the tai index (with wrapping) */

      if (++index >= SIMUART_BUFSIZE)
        {
          index = 0;
        }

      g_uarttail = index;
      sched_unlock();
      return ch;
    }
}

/****************************************************************************
 * Name: simuart_getc
 ****************************************************************************/

bool simuart_checkc(void)
{
  return g_uarthead != g_uarttail;
}

/****************************************************************************
 * Name: simuart_terminate
 ****************************************************************************/

void simuart_terminate(void)
{
  /* Restore the original terminal mode */

  (void)tcsetattr(0, TCSANOW, &g_cooked);
}
