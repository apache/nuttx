/****************************************************************************
 * include/nuttx/net/realtek_phy.h
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

#ifndef __INCLUDE_NUTTX_NET_REALTEK_PHY_H
#define __INCLUDE_NUTTX_NET_REALTEK_PHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: rtl8221b_register
 *
 * Description:
 *   Register the RTL8221B PHY driver with the PHYLIB framework.  Intended
 *   to be called once from board/MAC initialization, before connecting an
 *   RTL8221B PHY.  Registration is idempotent.
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PHY_REALTEK_RTL8221B
int rtl8221b_register(void);
#endif

#ifdef CONFIG_NET_PHY_REALTEK_RTL9000B
int rtl9000b_register(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_REALTEK_PHY_H */
