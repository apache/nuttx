/****************************************************************************
 * arch/arm/src/tiva/tiva_enableclks.h
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_ENABLECLKS_H
#define __ARCH_ARM_SRC_TIVA_TIVA_ENABLECLKS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Include chip specific definitions */

#if defined(CONFIG_ARCH_CHIP_LM3S) || defined(CONFIG_ARCH_CHIP_LM4F) || \
    defined(CONFIG_ARCH_CHIP_TM4C)
#  include "common/lmxx_tm4c_enableclks.h"
#elif defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  include "cc13xx/cc13xx_enableclks.h"
#else
#  error "Unsupported Tiva/Stellaris/SimpleLink clock controls"
#endif

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_ENABLECLKS_H */
