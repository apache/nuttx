/****************************************************************************
 * arch/arm/src/stm32/stm32_comp.c
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

#include "chip.h"

/* This file is only a thin shell that includes the correct COMP
 * implementation. At this moment only STM32 COMP IP version 1 device is
 * suportted.
 *   - STM32 COMP IP version 1: SMT32F33XX
 */

#if defined(CONFIG_STM32_HAVE_IP_COMP_V1)
#  include "stm32_comp_v1.c"
#else
#  error "STM32 COMP IP version not supported."
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/