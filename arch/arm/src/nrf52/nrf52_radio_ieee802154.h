/****************************************************************************
 * arch/arm/src/nrf52/nrf52_radio_ieee802154.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_RADIO_IEEE802154_H
#define __ARCH_ARM_SRC_NRF52_NRF52_RADIO_IEEE802154_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wireless/ieee802154/ieee802154_radio.h>

#include "nrf52_radio.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_radioi8_register
 *
 * Description:
 *   Register NRF52 radio in IEEE802154 mode
 *
 ****************************************************************************/

struct ieee802154_radio_s *
nrf52_radioi8_register(struct nrf52_radio_board_s *board);

#endif /* __ARCH_ARM_SRC_NRF52_NRF52_RADIO_IEEE802154_H */
