/****************************************************************************
 * drivers/crypto/se05x_internal.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_CRYPTO_SE05X_INTERNAL_H_
#define __INCLUDE_NUTTX_DRIVERS_CRYPTO_SE05X_INTERNAL_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/mutex.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;
struct pnt_handle;
struct se05x_config_s;

struct se05x_dev_s
{
  FAR struct se05x_config_s *config;
  FAR struct i2c_master_s *i2c; /* I2C interface */
  FAR struct pnt_handle *pnt;
  mutex_t mutex;
};

#endif /* __INCLUDE_NUTTX_DRIVERS_CRYPTO_SE05X_INTERNAL_H_ */
