/****************************************************************************
 * arch/sim/src/sim/win/sim_hosttime.c
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
#include <stdbool.h>
#include <windows.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define POW10_9 (1000000000ull)

/* Number of 100ns-seconds between the beginning of the Windows epoch
 * (Jan. 1, 1601) and the Unix epoch (Jan. 1, 1970)
 */

#define DELTA_EPOCH_IN_100NS  (0x19db1ded53e8000ull)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_gettime
 ****************************************************************************/

uint64_t host_gettime(bool rtc)
{
  static long long int ticks_per_sec;
  static uint64_t start;
  uint64_t current;
  LARGE_INTEGER now;
  FILETIME ftime;

  if (rtc)
    {
      GetSystemTimeAsFileTime(&ftime);

      return (((uint64_t)ftime.dwHighDateTime << 32 |
               ftime.dwLowDateTime) - DELTA_EPOCH_IN_100NS) * 100;
    }

  if (ticks_per_sec == 0)
    {
      QueryPerformanceFrequency(&now);
      InterlockedExchange64(&ticks_per_sec, now.QuadPart);
    }

  QueryPerformanceCounter(&now);

  current = now.QuadPart / ticks_per_sec * POW10_9 +
            (now.QuadPart % ticks_per_sec) * POW10_9 / ticks_per_sec;

  if (start == 0)
    {
      start = current;
    }

  return current - start;
}

/****************************************************************************
 * Name: host_sleep
 ****************************************************************************/

void host_sleep(uint64_t nsec)
{
  LARGE_INTEGER due;
  HANDLE timer;

  /* Convert to 100 nanosecond interval,
   * negative value indicates relative time
   */

  due.QuadPart = -((nsec + 99) / 100);

  timer = CreateWaitableTimer(NULL, TRUE, NULL);
  if (timer != NULL)
    {
      SetWaitableTimer(timer, &due, 0, NULL, NULL, 0);
      WaitForSingleObject(timer, INFINITE);
      CloseHandle(timer);
    }
}

/****************************************************************************
 * Name: host_sleepuntil
 ****************************************************************************/

void host_sleepuntil(uint64_t nsec)
{
  uint64_t now;

  now = host_gettime(false);
  if (nsec > now)
    {
      host_sleep(nsec - now);
    }
}

/****************************************************************************
 * Name: host_settimer
 *
 * Description:
 *   Set up a timer to send periodic signals.
 *
 * Input Parameters:
 *   nsec - timer expire time
 *
 * Returned Value:
 *   On success, (0) zero value is returned, otherwise a negative value.
 *
 ****************************************************************************/

int host_settimer(uint64_t nsec)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: host_timerirq
 *
 * Description:
 *   Get timer irq
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, irq num returned, otherwise a negative value.
 *
 ****************************************************************************/

int host_timerirq(void)
{
  return 0;
}
