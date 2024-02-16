/****************************************************************************
 * boards/arm/nrf52/common/include/nrf52_mrf24j40.h
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

#ifndef __BOARDS_ARM_NRF52_COMMON_INCLUDE_NRF52_MRF24J40_H
#define __BOARDS_ARM_NRF52_COMMON_INCLUDE_NRF52_MRF24J40_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/wireless/ieee802154/mrf24j40.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct nrf52_mrf24j40_s
{
  struct mrf24j40_lower_s  dev;
  xcpt_t                   handler;
  void                    *arg;
  uint32_t                 intcfg;
  uint8_t                  spidev;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_mrf24j40_devsetup
 *
 * Description:
 *   Initialize one the MRF24J40 device in one mikroBUS slot
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_IEEE802154_MRF24J40
int nrf52_mrf24j40_devsetup(struct nrf52_mrf24j40_s *priv);
#endif

#endif /* __BOARDS_ARM_NRF52_COMMON_INCLUDE_NRF52_MRF24J40_H */
