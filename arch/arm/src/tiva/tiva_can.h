/****************************************************************************
 * arch/arm/src/tiva/tiva_can.h
 * Classic (character-device) lower-half driver for the Tiva CAN modules.
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_CAN_H
#define __ARCH_ARM_SRC_TIVA_TIVA_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/can/can.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_CAN) && (defined(CONFIG_TIVA_CAN0) || defined(CONFIG_TIVA_CAN1))

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
#ifdef CONFIG_TIVA_CHAR_DEV_CAN
/****************************************************************************
 * Name: tiva_can_initialize
 *
 * Description:
 *   Initialize the selected CAN module
 *
 * Input Parameters:
 *   Device path, a string of the form "/dev/can0" or "/dev/can1"
 *   Module number, for chips with multiple modules (typically 0 or 1)
 *
 * Returned Value:
 *   Pointer to can_dev_s (CAN device structure), or NULL on failure.
 *
 ****************************************************************************/

int tiva_can_initialize(char *devpath, int modnum);
#elif CONFIG_TIVA_SOCKET_CAN
/****************************************************************************
 * Name: tiva_cansockinitialize
 *
 * Description:
 *   Initialize the selected CAN module using socket net API
 *
 * Input Parameters:
 *   Module number, for chips with multiple modules (typically 0 or 1)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int tiva_cansockinitialize(int modnum);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_CAN && (CONFIG_TIVA_CAN0 || CONFIG_TIVA_CAN1) */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_CAN_H */
