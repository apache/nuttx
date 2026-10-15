/****************************************************************************
 * net/bt/compat/src/kernel/queue.c
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* SPDX-License-Identifier: Apache-2.0 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/compat/kernel.h>
#include <errno.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void *k_fifo_remove_locked(struct k_fifo *fifo)
{
  return list_remove_head(&fifo->data_q);
}

static void k_poll_register_fifo_events(struct k_poll_event *events,
                                        int num_events)
{
  int i;

  for (i = 0; i < num_events; i++)
    {
      if (events[i].type == K_POLL_TYPE_FIFO_DATA_AVAILABLE &&
          events[i].fifo != NULL)
        {
          irqstate_t flags;

          flags = spin_lock_irqsave(&events[i].fifo->lock);
          events[i].fifo->poll_waiters++;
          spin_unlock_irqrestore(&events[i].fifo->lock, flags);
        }
    }
}

static void k_poll_unregister_fifo_events(struct k_poll_event *events,
    int num_events)
{
  int i;

  for (i = 0; i < num_events; i++)
    {
      if (events[i].type == K_POLL_TYPE_FIFO_DATA_AVAILABLE &&
          events[i].fifo != NULL)
        {
          irqstate_t flags;

          flags = spin_lock_irqsave(&events[i].fifo->lock);
          if (events[i].fifo->poll_waiters > 0)
            {
              events[i].fifo->poll_waiters--;
            }

          spin_unlock_irqrestore(&events[i].fifo->lock, flags);
        }
    }
}

static int k_poll_check_fifo_events(struct k_poll_event *events,
                                    int num_events)
{
  int i;

  for (i = 0; i < num_events; i++)
    {
      events[i].state = K_POLL_STATE_NOT_READY;

      if (events[i].type == K_POLL_TYPE_FIFO_DATA_AVAILABLE &&
          events[i].fifo != NULL)
        {
          irqstate_t flags;
          bool ready;

          flags = spin_lock_irqsave(&events[i].fifo->lock);
          if (events[i].fifo->poll_cancel_waiters > 0)
            {
              events[i].fifo->poll_cancel_waiters--;
              events[i].state = K_POLL_STATE_CANCELLED;
              spin_unlock_irqrestore(&events[i].fifo->lock,
                                     flags);
              return 0;
            }

          ready = !list_is_empty(&events[i].fifo->data_q);
          spin_unlock_irqrestore(&events[i].fifo->lock, flags);

          if (ready)
            {
              events[i].state =
                K_POLL_STATE_FIFO_DATA_AVAILABLE;
              return 0;
            }
        }
    }

  return -EAGAIN;
}

void k_fifo_init(struct k_fifo *fifo)
{
  list_initialize(&fifo->data_q);
  nxsem_init(&fifo->sem, 0, 0);
  fifo->lock = SP_UNLOCKED;
  fifo->waiters = 0;
  fifo->poll_waiters = 0;
  fifo->cancel_waiters = 0;
  fifo->poll_cancel_waiters = 0;
}

void k_fifo_cancel_wait(struct k_fifo *fifo)
{
  irqstate_t flags;
  unsigned int waiters;
  unsigned int i;

  flags = spin_lock_irqsave(&fifo->lock);
  waiters = fifo->waiters;
  fifo->cancel_waiters += waiters;
  fifo->poll_cancel_waiters += fifo->poll_waiters;
  spin_unlock_irqrestore(&fifo->lock, flags);

  for (i = 0; i < waiters; i++)
    {
      nxsem_post(&fifo->sem);
    }
}

void k_fifo_put(struct k_fifo *fifo, void *data)
{
  irqstate_t flags = spin_lock_irqsave(&fifo->lock);

  list_add_tail(&fifo->data_q, (struct list_node *)data);
  spin_unlock_irqrestore(&fifo->lock, flags);
  nxsem_post(&fifo->sem);
}

void *k_fifo_get(struct k_fifo *fifo, clock_t timeout)
{
  irqstate_t flags;
  struct list_node *node;
  clock_t deadline = 0;
  int ret;

  if (timeout > 0)
    {
      deadline = clock_systime_ticks() + timeout;
    }

  while (1)
    {
      flags = spin_lock_irqsave(&fifo->lock);
      node = k_fifo_remove_locked(fifo);
      spin_unlock_irqrestore(&fifo->lock, flags);

      if (node != NULL)
        {
          (void)nxsem_trywait(&fifo->sem);
          return node;
        }

      if (timeout == 0)
        {
          return NULL;
        }

      flags = spin_lock_irqsave(&fifo->lock);
      fifo->waiters++;
      spin_unlock_irqrestore(&fifo->lock, flags);

      if (timeout < 0)
        {
          ret = nxsem_wait_uninterruptible(&fifo->sem);
        }
      else
        {
          clock_t now = clock_systime_ticks();

          if ((clock_t)(deadline - now) <= 0)
            {
              ret = -ETIMEDOUT;
            }
          else
            {
              ret = nxsem_tickwait_uninterruptible(&fifo->sem,
                                                   deadline - now);
            }
        }

      flags = spin_lock_irqsave(&fifo->lock);
      if (fifo->waiters > 0)
        {
          fifo->waiters--;
        }

      if (ret >= 0 && fifo->cancel_waiters > 0)
        {
          fifo->cancel_waiters--;
          spin_unlock_irqrestore(&fifo->lock, flags);
          return NULL;
        }

      spin_unlock_irqrestore(&fifo->lock, flags);

      if (ret < 0)
        {
          return NULL;
        }
    }
}

bool k_fifo_is_empty(struct k_fifo *fifo)
{
  irqstate_t flags = spin_lock_irqsave(&fifo->lock);
  bool empty = list_is_empty(&fifo->data_q);

  spin_unlock_irqrestore(&fifo->lock, flags);
  return empty;
}

void *k_fifo_peek_head(struct k_fifo *fifo)
{
  irqstate_t flags = spin_lock_irqsave(&fifo->lock);
  struct list_node *node = list_peek_head(&fifo->data_q);

  spin_unlock_irqrestore(&fifo->lock, flags);
  return node;
}

void z_k_queue_prepend_fifo(struct k_fifo *fifo, void *data)
{
  irqstate_t flags = spin_lock_irqsave(&fifo->lock);

  list_add_head(&fifo->data_q, (struct list_node *)data);
  spin_unlock_irqrestore(&fifo->lock, flags);
  nxsem_post(&fifo->sem);
}

bool z_k_queue_remove_fifo(struct k_fifo *fifo, void *data)
{
  irqstate_t flags;
  bool removed;

  flags = spin_lock_irqsave(&fifo->lock);
  removed = list_find_and_remove(&fifo->data_q,
                                 (struct list_node *)data);
  spin_unlock_irqrestore(&fifo->lock, flags);

  if (removed)
    {
      (void)nxsem_trywait(&fifo->sem);
    }

  return removed;
}

int k_poll(struct k_poll_event *events, int num_events, clock_t timeout)
{
  clock_t deadline = 0;
  int ret;

  ret = k_poll_check_fifo_events(events, num_events);
  if (ret == 0 || timeout == 0)
    {
      return ret;
    }

  if (timeout > 0)
    {
      deadline = clock_systime_ticks() + timeout;
    }

  k_poll_register_fifo_events(events, num_events);

  while (1)
    {
      ret = k_poll_check_fifo_events(events, num_events);
      if (ret == 0)
        {
          break;
        }

      if (timeout > 0 &&
          (clock_t)clock_systime_ticks() >= deadline)
        {
          ret = -EAGAIN;
          break;
        }

      nxsched_msleep(1);
    }

  k_poll_unregister_fifo_events(events, num_events);

  return ret;
}
