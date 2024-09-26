/****************************************************************************
 * drivers/i3c/internals.h
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

#ifndef  __INCLUDE_NUTTX_I3C_INTERNALS_H
#define  __INCLUDE_NUTTX_I3C_INTERNALS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/i3c/master.h>

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

void i3c_bus_normaluse_lock(FAR struct i3c_bus *bus);
void i3c_bus_normaluse_unlock(FAR struct i3c_bus *bus);

FAR struct i3c_master_controller *
    i3c_adapter_to_i3c_master(FAR struct i3c_master_s *i3c_master);

int i3c_dev_do_priv_xfers_locked(FAR struct i3c_dev_desc *dev,
                                 FAR struct i3c_priv_xfer *xfers,
                                 int nxfers);
int i3c_dev_disable_ibi_locked(FAR struct i3c_dev_desc *dev);
int i3c_dev_enable_ibi_locked(FAR struct i3c_dev_desc *dev);
int i3c_dev_request_ibi_locked(FAR struct i3c_dev_desc *dev,
                               FAR const struct i3c_ibi_setup *req);
void i3c_dev_free_ibi_locked(FAR struct i3c_dev_desc *dev);

#endif /*  __INCLUDE_NUTTX_I3C_INTERNAL_H */
