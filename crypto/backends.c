/****************************************************************************
 * crypto/backends.c
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

#include <crypto/cryptodev.h>
#include <crypto/cryptosoft.h>
#include <nuttx/drivers/drivers.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: crypto_backends_init
 *
 * Description:
 *   Put the crypto backends into the driver table, making them reachable
 *   through crypto_newsession() / crypto_invoke(). The /dev/crypto interface
 *   is one user of those backends rather than a prerequisite for them, so it
 *   is registered separately by devcrypto_register(): a build whose only
 *   crypto users are in the kernel needs this function and not the character
 *   device.
 *
 ****************************************************************************/

void crypto_backends_init(void)
{
#ifdef CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_CRYPTO
  swcr_init();
#endif

#ifdef CONFIG_CRYPTO_CRYPTODEV_SOFTWARE_KEYMGMT
  swkey_init();
#endif

#ifdef CONFIG_CRYPTO_CRYPTODEV_HARDWARE
  hwcr_init();
#endif
}
