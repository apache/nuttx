/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_utils.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_UTILS_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_UTILS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

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

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_UTILS_H */
