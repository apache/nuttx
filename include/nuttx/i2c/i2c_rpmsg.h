/****************************************************************************
 * include/nuttx/i2c/i2c_rpmsg.h
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

#ifndef __INCLUDE_NUTTX_I2C_I2C_RPMSG_H
#define __INCLUDE_NUTTX_I2C_I2C_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef CONFIG_I2C_RPMSG
/****************************************************************************
 * Name: i2c_rpmsg_initialize
 *
 * Description:
 *   Create an I2C client lower-half that forwards transfers to a remote
 *   RPMsg I2C server.
 *
 * Input Parameters:
 *   cpuname - Name of the remote CPU to connect to.
 *   name    - RPMsg I2C service name.
 *
 * Returned Value:
 *   A pointer to the I2C lower-half on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *i2c_rpmsg_initialize(FAR const char *cpuname,
                                              FAR const char *name);
#endif

#ifdef CONFIG_I2C_RPMSG_SERVER
/****************************************************************************
 * Name: i2c_rpmsg_server_initialize
 *
 * Description:
 *   Export a local I2C lower-half to remote RPMsg I2C clients.
 *
 * Input Parameters:
 *   name - RPMsg I2C service name.
 *   i2c  - Local I2C lower-half to export.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int i2c_rpmsg_server_initialize(FAR const char *name,
                                FAR struct i2c_master_s *i2c);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_I2C_I2C_RPMSG_H */
