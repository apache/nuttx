/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_lirc.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_LIRC_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_LIRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "esp_rmt.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

#if defined(CONFIG_ESP_RMT) && defined(CONFIG_DRIVERS_RC)

int esp_lirc_rx_initialize(int devno, FAR struct rmt_dev_s *rmt);
int esp_lirc_tx_initialize(int devno, FAR struct rmt_dev_s *rmt);

#endif /* CONFIG_ESP_RMT && CONFIG_DRIVERS_RC */

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_LIRC_H */
