/****************************************************************************
 * include/nuttx/devfreq/devfreq_qos.h
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

#ifndef __INCLUDE_NUTTX_DEVFREQ_DEVFREQ_QOS_H
#define __INCLUDE_NUTTX_DEVFREQ_DEVFREQ_OQS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/plist.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum qos_req_type_e
{
  QOS_REQ_MIN,
  QOS_REQ_MAX
};

struct qos_request_s
{
  struct plist_node min_req;
  struct plist_node max_req;
};

struct qos_constraints_s
{
  struct plist_head min_requests;
  struct plist_head max_requests;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void qos_constraints_init(FAR struct qos_constraints_s *constraints);
FAR struct qos_request_s *qos_add_request(FAR struct qos_constraints_s *qos,
                                          uint32_t min, uint32_t max);
int qos_remove_request(FAR struct qos_constraints_s *constraints,
                       FAR struct qos_request_s *req);
int qos_update_request(FAR struct qos_constraints_s *constraints,
                       FAR struct qos_request_s *req,
                       uint32_t min, uint32_t max);
uint32_t qos_get_value(FAR struct qos_constraints_s *constraints,
                       enum qos_req_type_e type);
#endif
