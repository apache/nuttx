/****************************************************************************
 * drivers/wireless/bcm43xxx/ieee80211/bcmf_utils.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
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

#include <stdint.h>
#include <string.h>
#include <time.h>
#include <debug.h>
#include <stdio.h>
#include <queue.h>

#include "bcmf_utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LINE_LEN 16

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_hexdump
 ****************************************************************************/

void bcmf_hexdump(uint8_t *data, unsigned int len, unsigned long offset)
{
  unsigned int i;
  unsigned int char_count = 0;
  char char_line[20];
  char hex_line[64];

  for (i = 0; i < len; i++)
    {
      if (char_count >= LINE_LEN)
        {
          /* Flush line */

          wlinfo("%08x: %s%s\n",
                 offset + i - char_count, hex_line, char_line);
          char_count = 0;
        }

      sprintf(hex_line + 3 * char_count, "%02x ", data[i]);
      sprintf(char_line + char_count, "%c",
              data[i] < 0x20 || data[i] >= 0x7f? '.': data[i]);
      char_count++;
    }

  if (char_count > 0)
    {
      /* Flush last line */

      memset(hex_line + 3 * char_count, ' ', 3 * (LINE_LEN - char_count));
      hex_line[3 * LINE_LEN] = 0;
      wlinfo("%08x: %s%s\n", offset + i - char_count, hex_line, char_line);
    }
}

/****************************************************************************
 * Name: bcmf_sem_wait
 ****************************************************************************/

int bcmf_sem_wait(sem_t *sem, unsigned int timeout_ms)
{
  struct timespec abstime;
  unsigned int timeout_sec;

  /* Get the current time */

  clock_gettime(CLOCK_REALTIME, &abstime);

  timeout_sec      = timeout_ms / 1000;
  abstime.tv_sec  += timeout_sec;
  abstime.tv_nsec += 1000 * 1000 * (timeout_ms % 1000);

  if (abstime.tv_nsec >= 1000 * 1000 * 1000)
    {
      abstime.tv_sec++;
      abstime.tv_nsec -= 1000 * 1000 * 1000;
    }

  return nxsem_timedwait(sem, &abstime);
}

void bcmf_dqueue_push(dq_queue_t *queue, dq_entry_t *entry)
{
  if (queue->head == NULL)
    {
      /* List is empty */

      queue->tail = entry;

      entry->flink = entry;
      entry->blink = entry;
    }
  else
    {
      /* Insert entry at list head */

      entry->flink = queue->head;
      entry->blink = queue->tail;

      queue->head->blink = entry;
    }

  queue->head = entry;
}

dq_entry_t *bcmf_dqueue_pop_tail(dq_queue_t *queue)
{
  dq_entry_t *entry = queue->tail;

  if (queue->head == queue->tail)
    {
      /* List is empty */

      queue->head = NULL;
      queue->tail = NULL;
    }
  else
    {
      /* Pop from queue tail */

      queue->tail = entry->blink;
      entry->blink->flink = queue->head;
    }

  return entry;
}
