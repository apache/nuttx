/****************************************************************************
 * netutils/thttpd/timers.c
 * Simple Timer Routines
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Derived from the file of the same name in the original THTTPD package:
 *
 *   Copyright © 1995,1998,2000 by Jef Poskanzer <jef@mail.acme.com>.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sys/time.h>

#include <stdlib.h>
#include <stdio.h>
#include <debug.h>

#include "timers.h"

/****************************************************************************
 * Pre-Processor Definitons
 ****************************************************************************/

#define HASH_SIZE 67

/****************************************************************************
 * Private Data
 ****************************************************************************/

static Timer *timers[HASH_SIZE];
static Timer *free_timers;
static int alloc_count, active_count, free_count;

/****************************************************************************
 * Public Data
 ****************************************************************************/

ClientData JunkClientData;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static unsigned int hash(Timer *t)
{
  /* We can hash on the trigger time, even though it can change over the
   * life of a timer via the periodic bit.
   * This is because both of those guys call l_resort(), which  * recomputes
   * the hash and moves the timer to the appropriate list.
   */

  return ((unsigned int)t->time.tv_sec ^
          (unsigned int)t->time.tv_usec) % HASH_SIZE;
}

static void l_add(Timer * t)
{
  int h = t->hash;
  register Timer *t2;
  register Timer *t2prev;

  t2 = timers[h];
  if (t2 == NULL)
    {
      /* The list is empty. */
      timers[h] = t;
      t->prev = t->next = NULL;
    }
  else
    {
      if (t->time.tv_sec < t2->time.tv_sec ||
          (t->time.tv_sec == t2->time.tv_sec &&
           t->time.tv_usec <= t2->time.tv_usec))
        {
          /* The new timer goes at the head of the list. */

          timers[h] = t;
          t->prev = NULL;
          t->next = t2;
          t2->prev = t;
        }
      else
        {
          /* Walk the list to find the insertion point. */

          for (t2prev = t2, t2 = t2->next; t2 != NULL;
               t2prev = t2, t2 = t2->next)
            {
              if (t->time.tv_sec < t2->time.tv_sec ||
                  (t->time.tv_sec == t2->time.tv_sec &&
                   t->time.tv_usec <= t2->time.tv_usec))
                {
                  /* Found it. */
                  t2prev->next = t;
                  t->prev = t2prev;
                  t->next = t2;
                  t2->prev = t;
                  return;
                }
            }

          /* Oops, got to the end of the list.  Add to tail. */

          t2prev->next = t;
          t->prev = t2prev;
          t->next = NULL;
        }
    }
}

static void l_remove(Timer * t)
{
  int h = t->hash;

  if (t->prev == NULL)
    {
      timers[h] = t->next;
    }
  else
    {
      t->prev->next = t->next;
    }

  if (t->next != NULL)
    {
      t->next->prev = t->prev;
    }
}

static void l_resort(Timer * t)
{
  /* Remove the timer from its old list. */

  l_remove(t);

  /* Recompute the hash. */

  t->hash = hash(t);

  /* And add it back in to its new list, sorted correctly. */

  l_add(t);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tmr_init(void)
{
  int h;

  for (h = 0; h < HASH_SIZE; ++h)
    {
      timers[h] = NULL;
    }

  free_timers = NULL;
  alloc_count = active_count = free_count = 0;
}

Timer *tmr_create(struct timeval *nowP, TimerProc * timer_proc,
                  ClientData client_data, long msecs, int periodic)
{
  Timer *t;

  if (free_timers != NULL)
    {
      t = free_timers;
      free_timers = t->next;
      --free_count;
    }
  else
    {
      t = (Timer *) malloc(sizeof(Timer));
      if (!t)
        {
          return NULL;
        }
      ++alloc_count;
    }

  t->timer_proc = timer_proc;
  t->client_data = client_data;
  t->msecs = msecs;
  t->periodic = periodic;

  if (nowP != (struct timeval *)0)
    {
      t->time = *nowP;
    }
  else
    {
      (void)gettimeofday(&t->time, (struct timezone *)0);
    }

  t->time.tv_sec += msecs / 1000L;
  t->time.tv_usec += (msecs % 1000L) * 1000L;
  if (t->time.tv_usec >= 1000000L)
    {
      t->time.tv_sec += t->time.tv_usec / 1000000L;
      t->time.tv_usec %= 1000000L;
    }
  t->hash = hash(t);

  /* Add the new timer to the proper active list. */

  l_add(t);
  ++active_count;
  return t;
}

long tmr_mstimeout(struct timeval *nowP)
{
  int h;
  int gotone;
  long msecs, m;
  register Timer *t;

  gotone = 0;
  msecs = 0;                    /* make lint happy */

  /* Since the lists are sorted, we only need to look at the  * first timer on
   * each one.
   */

  for (h = 0; h < HASH_SIZE; ++h)
    {
      t = timers[h];
      if (t != NULL)
        {
          m = (t->time.tv_sec - nowP->tv_sec) * 1000L +
            (t->time.tv_usec - nowP->tv_usec) / 1000L;
          if (!gotone)
            {
              msecs = m;
              gotone = 1;
            }
          else if (m < msecs)
            {
              msecs = m;
            }
        }
    }
  if (!gotone)
    {
      return INFTIM;
    }

  if (msecs <= 0)
    {
      msecs = 0;
    }

  return msecs;
}

void tmr_run(struct timeval *nowP)
{
  int h;
  Timer *t;
  Timer *next;

  for (h = 0; h < HASH_SIZE; ++h)
    for (t = timers[h]; t != NULL; t = next)
      {
        next = t->next;

        /* Since the lists are sorted, as soon as we find a timer  * that isn't 
         * ready yet, we can go on to the next list
         */

        if (t->time.tv_sec > nowP->tv_sec ||
            (t->time.tv_sec == nowP->tv_sec && t->time.tv_usec > nowP->tv_usec))
          {
            break;
          }

        (t->timer_proc) (t->client_data, nowP);
        if (t->periodic)
          {
            /* Reschedule. */

            t->time.tv_sec += t->msecs / 1000L;
            t->time.tv_usec += (t->msecs % 1000L) * 1000L;
            if (t->time.tv_usec >= 1000000L)
              {
                t->time.tv_sec += t->time.tv_usec / 1000000L;
                t->time.tv_usec %= 1000000L;
              }
            l_resort(t);
          }
        else
          {
            tmr_cancel(t);
          }
      }
}

void tmr_cancel(Timer * t)
{
  /* Remove it from its active list. */

  l_remove(t);
  --active_count;

  /* And put it on the free list. */

  t->next = free_timers;
  free_timers = t;
  ++free_count;
  t->prev = NULL;
}

void tmr_cleanup(void)
{
  Timer *t;

  while (free_timers != NULL)
    {
      t = free_timers;
      free_timers = t->next;
      --free_count;
      free((void *)t);
      --alloc_count;
    }
}

void tmr_destroy(void)
{
  int h;

  for (h = 0; h < HASH_SIZE; ++h)
    {
      while (timers[h] != NULL)
        {
          tmr_cancel(timers[h]);
        }
    }
  tmr_cleanup();
}

/* Generate debugging statistics. */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_NET)
void tmr_logstats(long secs)
{
  ndbg("timers - %d allocated, %d active, %d free",
       alloc_count, active_count, free_count);

  if (active_count + free_count != alloc_count)
    {
      ndbg("ERROR: Timer counts don't add up!");
    }
}
#endif

