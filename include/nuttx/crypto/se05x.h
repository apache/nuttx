/****************************************************************************
 * include/nuttx/crypto/se05x.h
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

#ifndef __INCLUDE_NUTTX_CRYPTO_SE05X_H_
#define __INCLUDE_NUTTX_CRYPTO_SE05X_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#ifdef CONFIG_DEV_SE05X

#include <nuttx/fs/ioctl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SE05X_MODULE_UNIQUE_ID_LEN 18

#define SEIOC_GET_INFO _SEIOC(0x0000)         /* Arg: se05x_info_s */
#define SEIOC_GET_UID _SEIOC(0x0001)          /* Arg: se05x_uid_s */
#define SEIOC_GENERATE_KEYPAIR _SEIOC(0x0002)
/* Arg: se05x_key_store_entry_t */
#define SEIOC_SET_KEY _SEIOC(0x0003)          /* Arg: se05x_key_transmission_s */
#define SEIOC_SET_DATA _SEIOC(0x0004)         /* Arg: se05x_key_transmission_s */
#define SEIOC_GET_KEY _SEIOC(0x0005)          /* Arg: se05x_key_transmission_s */
#define SEIOC_GET_DATA _SEIOC(0x0006)         /* Arg: se05x_key_transmission_s */
#define SEIOC_DELETE_KEY _SEIOC(0x0007)       /* Arg: uint32_t key_id */
#define SEIOC_CREATE_SIGNATURE _SEIOC(0x0008) /* Arg: se05x_signature_s */
#define SEIOC_VERIFY_SIGNATURE _SEIOC(0x0009) /* Arg: se05x_signature_s */
#define SEIOC_DERIVE_SYMM_KEY _SEIOC(0x000A)  /* Arg: se05x_derive_key_s */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  SE05X_ASYM_CIPHER_EC_NIST_P_256 = 0,
} se05x_asym_cipher_type_e;

typedef enum
{
  SE05X_ALGORITHM_NA = 0,
  SE05X_ALGORITHM_PLAIN,
  SE05X_ALGORITHM_SHA,
  SE05X_ALGORITHM_SHA224,
  SE05X_ALGORITHM_SHA256,
  SE05X_ALGORITHM_SHA384,
  SE05X_ALGORITHM_SHA512,
  SE05X_ALGORITHM_SIZE,
} se05x_algorithm_e;

struct se05x_info_s
{
  uint16_t oef_id;
};

struct se05x_uid_s
{
  uint8_t uid[SE05X_MODULE_UNIQUE_ID_LEN];
};

struct se05x_generate_keypair_s
{
  uint32_t id;
  se05x_asym_cipher_type_e cipher;
};

struct se05x_key_store_entry_s
{
  uint32_t id;
  se05x_asym_cipher_type_e cipher;
};

struct se05x_buffer_s
{
  FAR uint8_t *buffer;
  size_t buffer_size;
  size_t buffer_content_size;
};

struct se05x_key_transmission_s
{
  struct se05x_key_store_entry_s entry;
  struct se05x_buffer_s content;
};

struct se05x_derive_key_s
{
  uint32_t private_key_id;
  uint32_t public_key_id;
  struct se05x_buffer_s content;
};

struct se05x_signature_s
{
  uint32_t key_id;
  se05x_algorithm_e algorithm;
  struct se05x_buffer_s tbs;
  struct se05x_buffer_s signature;
};

struct se05x_config_s
{
  uint8_t address;
  uint32_t frequency;

  CODE bool (*set_enable_pin)(bool state);
};

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: se05x_register
 *
 * Description:
 *   Register the se05x character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/se05x".
 *   i2c     - An I2C driver instance.
 *   config  - Pointer to the se05x configuration
 *
 * Returned Value:
 *   se050_status_success (0) on success,
 *       a negative value on failure
 *
 ****************************************************************************/

int se05x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   FAR struct se05x_config_s *config);

#endif /* CONFIG_DEV_SE05X */

#endif /* __INCLUDE_NUTTX_CRYPTO_SE05X_H_ */
