/****************************************************************************
 * wireless/ble_ll/ble_ll_hw.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>

#ifdef CONFIG_CRYPTO_SW_AES
#  include <crypto/aes.h>
#endif

#include "ble_ll_os.h"
#include "ble.h"
#include "ble_hw.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static ble_rng_isr_cb_t g_ble_hw_rng_cb;
static volatile bool g_ble_hw_rng_run;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_hw_encrypt_block
 *
 * Description:
 *   AES-128 encrypt one block.  The radio lower half is used when it
 *   provides an AES engine, the software AES library otherwise.
 *
 * Returned Value:
 *   0 on success, -1 on failure.
 *
 ****************************************************************************/

int ble_hw_encrypt_block(FAR struct ble_encryption_block *ecb)
{
#ifdef CONFIG_CRYPTO_SW_AES
  AES_CTX ctx;
#endif

  if (g_ble_ll_lower->ops->encrypt_block != NULL)
    {
      return BLE_PHY_OP(encrypt_block, ecb);
    }

#ifdef CONFIG_CRYPTO_SW_AES
  if (aes_setkey(&ctx, ecb->key, BLE_ENC_BLOCK_SIZE) != 0)
    {
      return -1;
    }

  aes_encrypt(&ctx, ecb->plain_text, ecb->cipher_text);
  explicit_bzero(&ctx, sizeof(ctx));
  return 0;
#else
  return -1;
#endif
}

/****************************************************************************
 * Name: ble_hw_rng_init
 *
 * Description:
 *   Register the callback receiving random bytes.
 *
 ****************************************************************************/

int ble_hw_rng_init(ble_rng_isr_cb_t cb, int bias)
{
  UNUSED(bias);
  DEBUGASSERT((g_ble_ll_lower->ops->rng_start == NULL) ==
              (g_ble_ll_lower->ops->rng_stop == NULL));

  g_ble_hw_rng_cb = cb;
  return 0;
}

/****************************************************************************
 * Name: ble_hw_rng_start
 *
 * Description:
 *   Start producing random bytes.  Without a hardware generator in the
 *   lower half, bytes from arc4random are delivered until the consumer
 *   stops the generator.
 *
 ****************************************************************************/

int ble_hw_rng_start(void)
{
  uint8_t rnum;

  if (g_ble_ll_lower->ops->rng_start != NULL)
    {
      return BLE_PHY_OP(rng_start);
    }

  g_ble_hw_rng_run = true;
  while (g_ble_hw_rng_run && g_ble_hw_rng_cb != NULL)
    {
      arc4random_buf(&rnum, sizeof(rnum));
      g_ble_hw_rng_cb(rnum);
    }

  return 0;
}

/****************************************************************************
 * Name: ble_hw_rng_stop
 *
 ****************************************************************************/

int ble_hw_rng_stop(void)
{
  if (g_ble_ll_lower->ops->rng_stop != NULL)
    {
      return BLE_PHY_OP(rng_stop);
    }

  g_ble_hw_rng_run = false;
  return 0;
}

/****************************************************************************
 * Name: ble_hw_get_public_addr
 *
 * Description:
 *   Get the public device address from the lower half.
 *
 * Returned Value:
 *   0 if a public address is present, -1 otherwise.
 *
 ****************************************************************************/

int ble_hw_get_public_addr(FAR ble_addr_t *addr)
{
  if (g_ble_ll_lower->ops->get_public_addr == NULL ||
      BLE_PHY_OP(get_public_addr, addr->val) != 0)
    {
      return -1;
    }

  addr->type = BLE_ADDR_PUBLIC;
  return 0;
}

/****************************************************************************
 * Name: ble_hw_get_static_addr
 *
 * Description:
 *   Get the random static device address from the lower half.
 *
 * Returned Value:
 *   0 if a static address is present, -1 otherwise.
 *
 ****************************************************************************/

int ble_hw_get_static_addr(FAR ble_addr_t *addr)
{
  if (g_ble_ll_lower->ops->get_static_addr == NULL ||
      BLE_PHY_OP(get_static_addr, addr->val) != 0)
    {
      return -1;
    }

  addr->type = BLE_ADDR_RANDOM;
  return 0;
}
