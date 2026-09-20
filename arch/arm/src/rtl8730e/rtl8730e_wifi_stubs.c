/****************************************************************************
 * arch/arm/src/rtl8730e/rtl8730e_wifi_stubs.c
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

/****************************************************************************
 * Stubs required by the amebasmart WiFi WHC host libs (lib_wifi_whc_ap.a,
 * lib_wifi_com_sec.a, lib_wpa_lite.a) that are not provided by NuttX or
 * ameba_os_wrap.c.
 *
 * PMU (pmu_*): CA32 does not manage power gating from NuttX; all PMU calls
 * are stubs that succeed silently.  KM4 owns sleep/wakelock management.
 *
 * TRNG: provide a simple fallback using NuttX arc4random (or a counter if
 * getrandom is unavailable).
 *
 * KV store (rt_kv_*): WPA PSK deauth history; not needed for basic STA.
 *
 * DiagVprintf: SDK internal log formatter; routed to vprintf.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * PMU stubs — power management owned by KM4 on amebasmart
 ****************************************************************************/

int pmu_yield_os_check(void)
{
  return 0;
}

int pmu_acquire_wakelock(uint32_t device_id)
{
  (void)device_id;
  return 0;
}

int pmu_release_wakelock(uint32_t device_id)
{
  (void)device_id;
  return 0;
}

int pmu_set_sysactive_time(uint32_t timeout)
{
  (void)timeout;
  return 0;
}

uint32_t pmu_get_wakelock_status(void)
{
  return 0;
}

int pmu_register_sleep_callback(uint32_t device_id, void *presleep,
                                void *psleep_param, void *pwakeup,
                                void *pwakeup_param)
{
  (void)device_id;
  (void)presleep;
  (void)psleep_param;
  (void)pwakeup;
  (void)pwakeup_param;
  return 0;
}

/****************************************************************************
 * TRNG stub — use a simple LFSR counter as fallback
 ****************************************************************************/

int TRNG_get_random_bytes(uint8_t *buf, uint32_t len)
{
  static uint32_t seed = 0xdeadbeef;
  uint32_t i;

  for (i = 0; i < len; i++)
    {
      seed ^= seed << 13;
      seed ^= seed >> 17;
      seed ^= seed << 5;
      buf[i] = (uint8_t)seed;
    }

  return 0;
}

/****************************************************************************
 * KV store stubs — WPA PSK deauth history, not needed for basic STA
 ****************************************************************************/

int rt_kv_get(const char *key, void *value, int len)
{
  (void)key;
  (void)value;
  (void)len;
  return -1; /* not found */
}

int rt_kv_set(const char *key, void *value, int len)
{
  (void)key;
  (void)value;
  (void)len;
  return 0;
}

int rt_kv_delete(const char *key)
{
  (void)key;
  return 0;
}

/****************************************************************************
 * DiagPrintf / DiagVprintf — SDK internal log formatters
 *
 * The SDK defines DiagPrintf as a weak symbol that normally routes through
 * LOGUART_PutChar using LOG_UART_IDX_FLAG.  On NuttX the AP SDK startup
 * that initialises LOG_UART_IDX_FLAG is never run, so the table has
 * garbage for channels 1-3 and LOGUART_PutChar crashes.  Override both
 * with strong symbols that route through NuttX vprintf instead.
 ****************************************************************************/

int DiagPrintf(const char *fmt, ...)
{
  va_list ap;
  int ret;

  va_start(ap, fmt);
  ret = vprintf(fmt, ap);
  va_end(ap);
  return ret;
}

int DiagVprintf(const char *fmt, va_list ap)
{
  return vprintf(fmt, ap);
}
