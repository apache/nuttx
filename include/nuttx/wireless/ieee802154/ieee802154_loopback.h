/****************************************************************************
 * include/nuttx/wireless/ieee802154/ieee802154_loopback.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_LOOPBACK_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_LOOPBACK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_IEEE802154_LOOPBACK

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ieee8021514_loopback
 *
 * Description:
 *   Initialize and register the Ieee802.15.4 MAC loopback network driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int ieee8021514_loopback(void);

#endif /* CONFIG_IEEE802154_LOOPBACK */
#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_LOOPBACK_H */
