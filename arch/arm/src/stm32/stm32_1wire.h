/****************************************************************************
 * arch/arm/src/stm32/stm32_1wire.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_1WIRE_H
#define __ARCH_ARM_SRC_STM32_STM32_1WIRE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "stm32_uart.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_1wireinitialize
 *
 * Description:
 *   Initialize the selected 1-Wire port. And return a unique instance of
 *   struct struct onewire_dev_s.  This function may be called to obtain
 *   multiple instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple 1-Wire interfaces)
 *
 * Returned Value:
 *   Valid 1-Wire device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct onewire_dev_s *stm32_1wireinitialize(int port);

/****************************************************************************
 * Name: stm32_1wireuninitialize
 *
 * Description:
 *   De-initialize the selected 1-Wire port, and power down the device.
 *
 * Input Parameters:
 *   Device structure as returned by the stm32_1wireinitialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int stm32_1wireuninitialize(struct onewire_dev_s *dev);

#endif /* __ARCH_ARM_SRC_STM32_STM32_1WIRE_H */
