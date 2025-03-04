/****************************************************************************
 * arch/arm/src/samv7/sam_1wire.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_1WIRE_H
#define __ARCH_ARM_SRC_SAMV7_SAM_1WIRE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_1wireinitialize
 *
 * Description:
 *   Initialize the selected 1-Wire port. And return a unique instance of
 *   struct onewire_dev_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple 1-Wire interfaces)
 *
 * Returned Value:
 *   Valid 1-Wire device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct onewire_dev_s *sam_1wireinitialize(int port);

#endif /* __ARCH_ARM_SRC_SAMV7_SAM_SERIAL_H */
