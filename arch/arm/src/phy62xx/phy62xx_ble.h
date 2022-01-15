/****************************************************************************
 * arch/arm/src/phy62xx/phy62xx_ble.h
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

#ifndef __ARCH_ARM_SRC_PHY62XX_PHY62XX_BLE_H
#define __ARCH_ARM_SRC_PHY62XX_PHY62XX_BLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

typedef uint8 llStatus_t;

int pplus_ble_initialize(void);

extern void osal_bm_free(void *payload_ptr);
extern uint8 osal_msg_deallocate(uint8 *msg_ptr);
extern void *HCI_bm_alloc(uint16 size);
extern llStatus_t LL_TxData(uint16 connId,
                            uint8 *pBuf,
                            uint8 pktLen,
                            uint8 fragFlag);

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_PHY62XX_PHY62XX_BLE_H */
