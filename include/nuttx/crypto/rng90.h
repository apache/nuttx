/****************************************************************************
 * include/nuttx/crypto/rng90.h
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

#ifndef __INCLUDE_NUTTX_CRYPTO_RNG90_H
#define __INCLUDE_NUTTX_CRYPTO_RNG90_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_DEV_RNG90)

#define RNG90_I2C_ADDR 0x40
#define RNG90_I2C_FREQ 400000

#define _RNG90IOCBASE   (0x3b00) /* RNG90 ioctl base */
#define _RNG90IOC(nr)   _IOC(_RNG90IOCBASE, nr)

#define RNG90_IOC_WAKEUP   _RNG90IOC(0x01)
#define RNG90_IOC_SLEEP    _RNG90IOC(0x02)
#define RNG90_IOC_GENRND   _RNG90IOC(0x03)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int rng90_register(FAR const char *devpath,
                   FAR struct i2c_master_s *i2c,
                   uint8_t addr);

#endif /* CONFIG_I2C && CONFIG_DEV_RNG90 */
#endif /* __INCLUDE_NUTTX_CRYPTO_RNG90_H */
