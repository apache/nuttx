/****************************************************************************
 * arch/arm/src/nrf52/sdc/core_cm4.h
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

#ifndef __ARCH_ARM_SRC_NRF52_SDC_CORE_CM4_H
#define __ARCH_ARM_SRC_NRF52_SDC_CORE_CM4_H

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* These are the definitions needed from CMSIS */

#ifdef __cplusplus
#  define   __I     volatile
#else
#  define   __I     volatile const
#endif
#define     __O     volatile
#define     __IO    volatile
#define     __IM    volatile const
#define     __OM    volatile
#define     __IOM   volatile

#endif /* __ARCH_ARM_SRC_NRF52_SDC_CORE_CM4_H */
