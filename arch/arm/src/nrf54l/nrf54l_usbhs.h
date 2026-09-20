/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_usbhs.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_NRF54L_USBHS_H
#define __ARCH_ARM_SRC_NRF54L_NRF54L_USBHS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdbool.h>

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* The USB device driver serializes these operations. The VBUS callback runs
 * in interrupt context and must defer controller start/stop to a worker.
 * After initialize(), inspect vbus() to detect an already connected cable.
 */

int nrf54l_usbhs_initialize(xcpt_t handler, void *arg);
void nrf54l_usbhs_uninitialize(void);
bool nrf54l_usbhs_vbus(void);

/* Enable the PHY and release core reset with the D+ pull-up held off.
 * Called from thread context; only a successful return permits DWC2 access.
 * Stop core register accesses and its IRQ before calling disable().
 */

int nrf54l_usbhs_enable(void);
void nrf54l_usbhs_disable(void);
int nrf54l_usbhs_pullup(bool enable);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_NRF54L_NRF54L_USBHS_H */
