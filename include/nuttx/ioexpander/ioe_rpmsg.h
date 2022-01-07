/****************************************************************************
 * include/nuttx/ioexpander/ioe_rpmsg.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_IOE_RPMSG_H
#define __INCLUDE_NUTTX_IOEXPANDER_IOE_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/ioexpander/ioexpander.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ioe_rpmsg_server_initialize
 *
 * Description:
 *   Initialize IO expander rpmsg server
 *
 ****************************************************************************/

int ioe_rpmsg_server_initialize(FAR const char *name,
                                FAR struct ioexpander_dev_s *ioe);

/****************************************************************************
 * Name: ioe_rpmsg_client_initialize
 *
 * Description:
 *   Initialize IO expander rpmsg client
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *
ioe_rpmsg_client_initialize(FAR const char *cpuname,
                            FAR const char *name);

#endif /* __INCLUDE_NUTTX_IOEXPANDER_IOE_RPMSG_H */
