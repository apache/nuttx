/****************************************************************************
 * arch/arm/src/rm57/rm57_i2c.h
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

#ifndef __ARCH_ARM_SRC_RM57_RM57_I2C_H
#define __ARCH_ARM_SRC_RM57_RM57_I2C_H

/* I2C master driver for the two I2C modules (SPNU562A chapter 30).
 *
 * What is implemented: interrupt-driven master transfers in 7-bit and
 * 10-bit addressing, repeated starts between messages, I2C_M_NOSTART
 * concatenation, address-only probes (zero-length writes), bus recovery
 * through the pins' GIO mode, and the module's digital loopback for a
 * self-test.
 *
 * What is not: slave mode, DMA (only I2C2 has request lines), multi-master
 * arbitration recovery beyond reporting -EAGAIN, and bit counts other than
 * eight.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

#include <nuttx/i2c/i2c_master.h>

#include "hardware/rm57_i2c.h"

#ifdef CONFIG_RM57_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Address the module answers to as its own slave while in digital
 * loopback; transfers to it are what the self-test sends.
 */

#define RM57_I2C_LOOPBACK_ADDR  0x2a

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: rm57_i2cbus_initialize
 *
 * Description:
 *   Bring one I2C module out of reset as a master at 100 kHz and return
 *   its lower half.  Calls are reference counted; later calls for the same
 *   bus return the same instance.
 *
 * Input Parameters:
 *   bus - Module number, 1 or 2
 *
 * Returned Value:
 *   The I2C master on success; NULL if the bus does not exist or is not
 *   enabled in the configuration.
 *
 ****************************************************************************/

struct i2c_master_s *rm57_i2cbus_initialize(int bus);

/****************************************************************************
 * Name: rm57_i2cbus_uninitialize
 *
 * Description:
 *   Drop one reference to an I2C master; the last one puts the module back
 *   into reset and detaches its interrupt.
 *
 * Input Parameters:
 *   dev - Instance returned by rm57_i2cbus_initialize()
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rm57_i2cbus_uninitialize(struct i2c_master_s *dev);

#ifdef CONFIG_RM57_I2C_LOOPBACK
/****************************************************************************
 * Name: rm57_i2c_loopback
 *
 * Description:
 *   Switch a module's digital loopback on or off.  While it is on, the
 *   module addresses itself at RM57_I2C_LOOPBACK_ADDR whatever address a
 *   message carries, and every byte that comes back through the receive
 *   register during a write is stored in the echo buffer, across messages
 *   and transfers, until the buffer is full.
 *
 * Input Parameters:
 *   dev     - Instance returned by rm57_i2cbus_initialize()
 *   echo    - Buffer for the looped-back bytes; NULL turns loopback off
 *   echolen - Size of the buffer
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rm57_i2c_loopback(struct i2c_master_s *dev, uint8_t *echo,
                      size_t echolen);

/****************************************************************************
 * Name: rm57_i2c_echocount
 *
 * Description:
 *   Return how many bytes have come back since loopback was switched on.
 *
 ****************************************************************************/

size_t rm57_i2c_echocount(struct i2c_master_s *dev);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_RM57_I2C */
#endif /* __ARCH_ARM_SRC_RM57_RM57_I2C_H */
