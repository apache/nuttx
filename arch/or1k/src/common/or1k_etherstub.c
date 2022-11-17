/****************************************************************************
 * arch/or1k/src/common/or1k_etherstub.c
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

#include <nuttx/config.h>

#include "or1k_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: or1k_etinitialize (stub)
 *
 * Description:
 *   This is a stub version of or1k_etinitialize. Normally, etinitialize
 *   is defined in board/xyz_network.c for board-specific Ethernet
 *   implementations, or chip/xyx_ethernet.c for chip-specific Ethernet
 *   implementations.  The stub version here is used in the corner case where
 *   the network is enable yet there is no Ethernet driver to be initialized.
 *   In this case, up_initialize will still try to call or1k_etinitialize()
 *   when one does not exist.  This corner case would occur if, for example,
 *   only a USB network interface is being used or perhaps if a SLIP is
 *   being used).
 *
 *   Use of this stub is deprecated.  The preferred mechanism is to use
 *   CONFIG_NETDEV_LATEINIT=y to suppress the call to or1k_etinitialize() in
 *   up_initialize().  Then this stub would not be needed.
 *
 ****************************************************************************/

void or1k_etinitialize(void)
{
}
