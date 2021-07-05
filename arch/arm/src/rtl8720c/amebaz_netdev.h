/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_netdev.h
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
 * Copyright(c) 2016 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 ****************************************************************************/

#ifndef __DRIVERS_WIRELESS_IEEE80211_AMEBAZ_AMEBAZ_NETDEV_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define __DRIVERS_WIRELESS_IEEE80211_AMEBAZ_AMEBAZ_NETDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "amebaz_driver.h"
#include "amebaz_wlan.h"

int   amebaz_netdev_register(FAR struct amebaz_dev_s *priv);
void  amebaz_netdev_notify_receive(FAR struct amebaz_dev_s *priv,
                                           int index, unsigned int len);
#endif /* __DRIVERS_WIRELESS_IEEE80211_AMEBAZ_AMEBAZ_NETDEV_H */
