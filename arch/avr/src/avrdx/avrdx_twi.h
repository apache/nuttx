/****************************************************************************
 * arch/avr/src/avrdx/avrdx_twi.h
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

#ifndef __ARCH_AVR_SRC_AVRDX_AVRDX_TWI_H
#define __ARCH_AVR_SRC_AVRDX_AVRDX_TWI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "avrdx_iodefs.h"
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Base address of TWIn peripheral. TWI index corresponds to the location
 * of its I/O registers in memory. Ignores its parameter if the chip
 * only has one TWI.
 */

#ifdef CONFIG_AVR_HAVE_TWI1
#  define AVRDX_TWI(n) (*(avr_twi_t *) (0x0900 + n * 0x20))
#else
#  define AVRDX_TWI(n) (*(avr_twi_t *) (0x0900 + 0 * 0x20))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_initialize_twi
 *
 * Description:
 *   Initializer for TWI device. Allocates data structures and configures
 *   the peripheral. May be called multiple times by multiple drivers using
 *   the bus.
 *
 * Input Parameters:
 *   Peripheral number, ignored if the chip only has single TWI
 *
 * Returned Value:
 *   Initialized structure cast to i2c_master_s
 *
 ****************************************************************************/

FAR struct i2c_master_s *avrdx_initialize_twi(uint8_t twi_n);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVRDX_AVRDX_TWI_H */
