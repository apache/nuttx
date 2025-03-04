/****************************************************************************
 * arch/arm/src/nrf53/nrf53_spu.h
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

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_SPU_H
#define __ARCH_ARM_SRC_NRF53_NRF53_SPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPU only for secure app core image */

#if defined(CONFIG_NRF53_APPCORE) && \
    !defined(CONFIG_ARCH_TRUSTZONE_NONSECURE)
#  define HAVE_SPU_CONFIG
#else
#  undef HAVE_SPU_CONFIG
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifdef HAVE_SPU_CONFIG
/****************************************************************************
 * Name: nrf53_spu_configure
 ****************************************************************************/

void nrf53_spu_configure(void);
#endif

#endif /* __ARCH_ARM_SRC_NRF53_NRF53_SPU_H */
