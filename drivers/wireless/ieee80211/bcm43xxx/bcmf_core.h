/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_core.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_CORE_H
#define __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_CORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "bcmf_interface.h"

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

int bcmf_read_sbreg(FAR bcmf_interface_dev_t *ibus, uint32_t address,
                          uint8_t *reg, unsigned int len);

int bcmf_write_sbreg(FAR bcmf_interface_dev_t *ibus, uint32_t address,
                          uint8_t *reg, unsigned int len);

bool bcmf_core_isup(FAR bcmf_interface_dev_t *ibus, unsigned int core);

void bcmf_core_disable(FAR bcmf_interface_dev_t *ibus,
                       unsigned int core,
                       uint32_t prereset,
                       uint32_t reset);

void bcmf_core_reset(FAR bcmf_interface_dev_t *ibus,
                     unsigned int core,
                     uint32_t prereset,
                     uint32_t reset,
                     uint32_t postreset);

int bcmf_core_upload_firmware(FAR bcmf_interface_dev_t *ibus);

static inline int bcmf_read_sbregb(FAR bcmf_interface_dev_t *bus,
                                   uint32_t                  address,
                                   uint8_t                  *value)
{
  return bcmf_read_sbreg(bus, address, value, 1);
}

static inline int bcmf_read_sbregw(FAR bcmf_interface_dev_t *bus,
                                   uint32_t                  address,
                                   void                     *value)
{
  return bcmf_read_sbreg(bus, address, (uint8_t *)value, 4);
}

static inline int bcmf_write_sbregb(FAR bcmf_interface_dev_t *ibus,
                                    uint32_t address, uint8_t reg)
{
    return bcmf_write_sbreg(ibus, address, &reg, 1);
}

static inline int bcmf_write_sbregw(FAR bcmf_interface_dev_t *ibus,
                                    uint32_t address, uint32_t reg)
{
    return bcmf_write_sbreg(ibus, address, (uint8_t *)&reg, 4);
}

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCM43XXX_BCMF_CORE_H */
