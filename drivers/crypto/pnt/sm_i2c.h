/****************************************************************************
 * drivers/crypto/pnt/sm_i2c.h
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

/* Copyright 2023 NXP */

#ifndef __INCLUDE_NUTTX_CRYPTO_PNT_SM_I2C_H_
#define __INCLUDE_NUTTX_CRYPTO_PNT_SM_I2C_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "se05x_types.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define axI2CInit sm_i2c_init
#define axI2CTerm sm_i2c_term
#define axI2CWrite sm_i2c_write
#define axI2CRead sm_i2c_read

#define I2C_IDLE 0
#define I2C_STARTED 1
#define I2C_RESTARTED 2
#define I2C_REPEATED_START 3
#define DATA_ACK 4
#define DATA_NACK 5
#define I2C_BUSY 6
#define I2C_NO_DATA 7
#define I2C_NACK_ON_ADDRESS 8
#define I2C_NACK_ON_DATA 9
#define I2C_ARBITRATION_LOST 10
#define I2C_TIME_OUT 11
#define I2C_OK 12
#define I2C_FAILED 13
#define I2C_BUS_0 (0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef unsigned int i2c_error_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

i2c_error_t sm_i2c_init(FAR void **conn_ctx, FAR const char *dev_name);
void sm_i2c_term(FAR void *conn_ctx, int mode);
i2c_error_t sm_i2c_write(FAR void *conn_ctx, unsigned char bus,
                         unsigned char addr, FAR unsigned char *tx,
                         unsigned short tx_len);
i2c_error_t sm_i2c_read(FAR void *conn_ctx, unsigned char bus,
                        unsigned char addr, FAR unsigned char *rx,
                        unsigned short rx_len);

#endif /* __INCLUDE_NUTTX_CRYPTO_PNT_SM_I2C_H_ */
