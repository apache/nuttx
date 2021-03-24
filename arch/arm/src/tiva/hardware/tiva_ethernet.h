/****************************************************************************
 * arch/arm/src/tiva/hardware/tiva_ethernet.h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_ETHERNET_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_TM4C)
#  include "hardware/tm4c/tm4c_ethernet.h"
#elif defined(CONFIG_ARCH_CHIP_LM3S)
#  include "hardware/lm/lm3s_ethernet.h"
#endif

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_ETHERNET_H */
