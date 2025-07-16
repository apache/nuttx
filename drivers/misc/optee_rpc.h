/****************************************************************************
 * drivers/misc/optee_rpc.h
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

#ifndef __DRIVERS_MISC_OPTEE_RPC_H
#define __DRIVERS_MISC_OPTEE_RPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "optee.h"

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

void optee_rpc_handle_cmd(FAR struct optee_priv_data *priv,
                          FAR struct optee_shm *shm,
                          FAR void **last_page_list);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_MISC_OPTEE_RPC_H */
