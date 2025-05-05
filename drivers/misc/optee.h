/****************************************************************************
 * drivers/misc/optee.h
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

#ifndef __DRIVERS_MISC_OPTEE_H
#define __DRIVERS_MISC_OPTEE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/tee.h>

#include "optee_msg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPTEE_SERVER_PATH              "optee"
#define OPTEE_MAX_PARAM_NUM            6

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct optee_priv_data
{
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int optee_transport_init(void);
int optee_transport_open(FAR struct optee_priv_data **priv);
void optee_transport_close(FAR struct optee_priv_data *priv);
int optee_transport_call(FAR struct optee_priv_data *priv,
                         FAR struct optee_msg_arg *arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_MISC_OPTEE_H */
