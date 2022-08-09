/****************************************************************************
 * arch/arm/include/rp2040/i2c_slave.h
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

#ifndef __ARCH_ARM_INCLUDE_RP2040_I2C_SLAVE_H
#define __ARCH_ARM_INCLUDE_RP2040_I2C_SLAVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_slave.h>

#ifndef __ASSEMBLY__
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* There is no driver for I2C slave operations.  To create an I2C slave,
 * include this file (as: <arch/chip/i2c_slave.h>) and use either
 * rp2040_i2c0_slave_initialize or rp2040_i2c1_slave_initialize to
 * initialize the I2C for slave operations.
 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_i2c0_slave_initialize
 *
 * Description:
 *   Initialize I2C controller zero for slave operation, and return a pointer
 *   to the instance of struct i2c_slave_s.  This function should only be
 *   called once of a give controller.
 *
 *   Note: the same port cannot be initalized as both master and slave.
 *
 * Input Parameters:
 *   rx_buffer     - Buffer for data transmitted to us by an I2C master.
 *   rx_buffer_len - Length of rx_buffer.
 *   callback      - Callback function called when messages are received.
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RP2040_I2C0_SLAVE

struct i2c_slave_s *rp2040_i2c0_slave_initialize
                           (uint8_t              *rx_buffer,
                            size_t                rx_buffer_len,
                            i2c_slave_callback_t *callback);

#endif

/****************************************************************************
 * Name: rp2040_i2c1_slave_initialize
 *
 * Description:
 *   Initialize I2C controller zero for slave operation, and return a pointer
 *   to the instance of struct i2c_slave_s.  This function should only be
 *   called once of a give controller.
 *
 *   Note: the same port cannot be initalized as both master and slave.
 *
 * Input Parameters:
 *   rx_buffer     - Buffer for data transmitted to us by an I2C master.
 *   rx_buffer_len - Length of rx_buffer.
 *   callback      - Callback function called when messages are received.
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RP2040_I2C1_SLAVE

struct i2c_slave_s *rp2040_i2c1_slave_initialize
                           (uint8_t              *rx_buffer,
                            size_t                rx_buffer_len,
                            i2c_slave_callback_t *callback);

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_INCLUDE_RP2040_I2C_SLAVE_H */
