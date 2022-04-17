/****************************************************************************
 * arch/arm/src/kinetis/kinetis_usbotg.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_USBOTG_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_USBOTG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/kinetis_usbotg.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Buffer Descriptor Status Register layout. */

struct usbotg_bdtentry_s
{
  uint32_t status;  /* Status, byte count, and PID */
  uint8_t *addr;    /* Buffer address */
};
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

struct usbdev_s;
int kinetis_usbpullup(struct usbdev_s *dev,  bool enable);
void kinetis_usbsuspend(struct usbdev_s *dev, bool resume);
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_USBOTG_H */
