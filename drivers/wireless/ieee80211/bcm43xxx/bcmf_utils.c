/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_utils.c
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

          wlinfo("%08lx: %s%s\n",
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
      wlinfo("%08lx: %s%s\n", offset + i - char_count, hex_line, char_line);
    }
}

/****************************************************************************
 * Name: bcmf_sem_wait
 ****************************************************************************/

int bcmf_sem_wait(sem_t *sem, unsigned int timeout_ms)
{
  return nxsem_tickwait(sem, MSEC2TICK(timeout_ms));
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
