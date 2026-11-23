/****************************************************************************
 * include/nuttx/crypto/rpmsg_crypto.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
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

#ifndef __INCLUDE_NUTTX_CRYPTO_RPMSG_CRYPTO_H
#define __INCLUDE_NUTTX_CRYPTO_RPMSG_CRYPTO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: rpmsg_crypto_server_init
 *
 * Description:
 *   Rpmsg-crypto server initialize function, the server cpu should call
 *   this function.
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_RPMSG_SERVER
int rpmsg_crypto_server_init(void);
#endif

/****************************************************************************
 * Name: rpmsg_crypto_register
 *
 * Description:
 *   Rpmsg-crypto client initialize function, the client cpu should call
 *   this function in the board initialize process.
 *
 * Input Parameters:
 *   remotecpu  - the server cpu name
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_RPMSG
int rpmsg_crypto_register(FAR const char *remotecpu);
#endif

#ifdef __cplusplus
}
#endif
#endif /* __INCLUDE_NUTTX_CRYPTO_RPMSG_CRYPTO_H */
