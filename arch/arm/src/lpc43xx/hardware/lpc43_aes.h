/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_aes.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_AES_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_AES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The AES is controlled through a set of simple API calls located in the
 * LPC43xx ROM.  This value holds the pointer to the AES driver table.
 */

#define LPC43_ROM_AES_DRIVER_TABLE LPC43_ROM_DRIVER_TABLE2

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum lpc43_aescmd_e
{
  AES_API_CMD_ENCODE_ECB = 0,
  AES_API_CMD_DECODE_ECB = 1,
  AES_API_CMD_ENCODE_CBC = 2,
  AES_API_CMD_DECODE_CBC = 3
};

struct lpc43_aes_s
{
  /* Initialize the AES engine */

  void (*aes_init)(void);

  /* Offset 0x04 -- Defines AES engine operation mode.
   *  See enum lpc43_aescmd_e
   */

  unsigned int (*aes_set_mode)(unsigned int cmd);

  /* Load 128-bit AES user keys */

  void (*aes_load_key1)(void);
  void (*aes_load_key2)(void);

  /* Loads randomly generated key in AES engine. To update the RNG and load
   * a new random number, use the API call otp_GenRand before
   * aes_load_key_rng.
   */

  void (*aes_load_key_rng)(void);

  /* Loads 128-bit AES software defined user key (16 bytes) */

  void (*aes_load_key_sw)(const unsigned char *key);

  /* Loads 128-bit AES initialization vector (16 bytes) */

  void (*aes_load_iv_sw)(const unsigned char *iv);

  /* Loads 128-bit AES IC specific initialization vector, which is used to
   * decrypt a boot image.
   */

  void (*aes_load_iv_ic)(void);

  /* Process data */

  unsigned int (*aes_operate)(unsigned char *out,
                              const unsigned char *in, unsigned blocks);
};

enum lpc43_aes_errorcodes_e
{
  AES_API_ERR_BASE = 0x30000,
  AES_API_ERR_WRONG_CMD,
  AES_API_ERR_NOT_SUPPORTED,
  AES_API_ERR_KEY_ALREADY_PROGRAMMED,
  AES_API_ERR_DMA_CHANNEL_CFG,
  AES_API_ERR_DMA_MUX_CFG,
  AES_API_ERR_DMA_BUSY
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_AES_H */
