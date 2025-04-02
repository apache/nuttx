/****************************************************************************
 * drivers/devfreq/devfreq_qos.c
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

#include <nuttx/devfreq/devfreq_qos.h>
#include <nuttx/kmalloc.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qos_constraints_init
 *
 * Description:
 *   Initialize a qos_constraints_s struct.
 *
 * Input Parameters:
 *   qos - The qos constraints struct to initialize.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void qos_constraints_init(FAR struct qos_constraints_s *constraints)
{
  plist_head_init(&constraints->min_requests);
  plist_head_init(&constraints->max_requests);
}

/****************************************************************************
 * Name: qos_add_request
 *
 * Description:
 *   Add a qos request to qos constraints.
 *
 * Input Parameters:
 *   qos - The qos constraints to add the request to.
 *   min - The minimum priority/value of the request.
 *   max - The maximum priority/value of the request.
 *
 * Returned Value:
 *   A pointer to the new qos_request_s.
 *
 ****************************************************************************/

FAR struct qos_request_s *qos_add_request(
              FAR struct qos_constraints_s *constraints,
              uint32_t min, uint32_t max)
{
  FAR struct qos_request_s *req;

  if (!constraints)
    {
      return NULL;
    }

  req = kmm_zalloc(sizeof(struct qos_request_s));
  if (!req)
    {
      return NULL;
    }

  plist_node_init(&req->min_req, min);
  plist_node_init(&req->max_req, max);

  plist_add(&req->min_req, &constraints->min_requests);
  plist_add(&req->max_req, &constraints->max_requests);

  return req;
}

/****************************************************************************
 * Name: qos_remove_request
 *
 * Description:
 *   Remove qos request from qos constraints.
 *
 * Input Parameters:
 *   qos - The qos_constraints_s to remove the request from.
 *   req - The qos_request_s to remove.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure.
 *
 ****************************************************************************/

int qos_remove_request(FAR struct qos_constraints_s *constraints,
                       FAR struct qos_request_s *req)
{
  if (!req || !constraints)
    {
      return -EINVAL;
    }

  plist_del(&req->min_req, &constraints->min_requests);
  plist_del(&req->max_req, &constraints->max_requests);

  kmm_free(req);

  return 0;
}

/****************************************************************************
 * Name: qos_update_request
 *
 * Description:
 *   Update qos request.
 *
 * Input Parameters:
 *   qos - The qos constraints.
 *   req - The qos request to update.
 *   min - The new minimum priority/value of the request.
 *   max - The new maximum priority/value of the request.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure.
 *
 ****************************************************************************/

int qos_update_request(FAR struct qos_constraints_s *constraints,
                       FAR struct qos_request_s *req,
                       uint32_t min, uint32_t max)
{
  if (!req || !constraints)
    {
      return -EINVAL;
    }

  req->min_req.prio = min;
  req->max_req.prio = max;

  plist_del(&req->min_req, &constraints->min_requests);
  plist_del(&req->max_req, &constraints->max_requests);
  plist_add(&req->min_req, &constraints->min_requests);
  plist_add(&req->max_req, &constraints->max_requests);

  return 0;
}

/****************************************************************************
 * Name: qos_get_value
 *
 * Description:
 *   Get min or max value of qos constraints.
 *
 * Input Parameters:
 *   qos - The qos constraints.
 *   type - The type of request to get the value of.
 *
 * Returned Value:
 *   The value of the qos request.
 *
 ****************************************************************************/

uint32_t qos_get_value(FAR struct qos_constraints_s *constraints,
                       enum qos_req_type_e type)
{
  switch (type)
    {
      case QOS_REQ_MIN:
        {
          return plist_first(&constraints->min_requests)->prio;
        }

      case QOS_REQ_MAX:
        {
          return plist_last(&constraints->max_requests)->prio;
        }
    }

  return 0;
}
