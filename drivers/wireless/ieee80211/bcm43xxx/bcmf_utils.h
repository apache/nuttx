/****************************************************************************
 * drivers/wireless/bcm43xxx/ieee80211/bcmf_utils.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_UTILS_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_UTILS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <queue.h>

#include <nuttx/semaphore.h>

#define container_of(ptr, type, member) \
        (type *)((uint8_t *)(ptr) - offsetof(type, member))

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void bcmf_hexdump(uint8_t *data, unsigned int len, unsigned long offset);

int bcmf_sem_wait(sem_t *sem, unsigned int timeout_ms);

dq_entry_t *bcmf_dqueue_pop_tail(dq_queue_t *queue);
void bcmf_dqueue_push(dq_queue_t *queue, dq_entry_t *entry);

static inline uint16_t bcmf_getle16(void *val)
{
  uint8_t *valb = (uint8_t *)val;
  return (uint16_t)valb[0] << 8 | (uint16_t)valb[1];
}

static inline uint32_t bcmf_getle32(void *val)
{
  uint16_t *valw = (uint16_t *)val;
  return (uint32_t)bcmf_getle16(valw) << 16 | bcmf_getle16(valw + 1);
}

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_UTILS_H */
