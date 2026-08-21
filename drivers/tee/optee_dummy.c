/****************************************************************************
 * drivers/tee/optee_dummy.c
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

#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/drivers/optee.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Name of the channel this announces.  It is part of the interface between
 * the two operating systems rather than an internal constant, as the REE
 * matches it through the id_table of its own rpmsg TEE driver.
 */

#define OPTEE_DUMMY_EPT_NAME "rpmsg-tee"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: optee_dummy_ept_cb
 *
 * Description:
 *   Sink for the announcement endpoint.  Nothing is expected on the channel
 *   itself: the REE allocates the memory it shares with the TEE out of the
 *   rpmsg buffer pool and names the result in its own requests, which reach
 *   the TEE over the OP-TEE service socket rather than here.  A callback
 *   still has to be supplied, as rpmsg_create_ept() rejects a NULL one.
 *
 ****************************************************************************/

static int optee_dummy_ept_cb(FAR struct rpmsg_endpoint *ept,
                              FAR void *data, size_t len,
                              uint32_t src, FAR void *priv)
{
  return 0;
}

/****************************************************************************
 * Name: optee_dummy_ept_release
 *
 * Description:
 *   Free the endpoint once the last reference to it is gone.  Tearing the
 *   rpmsg device down destroys the endpoints attached to it, which is what
 *   drops that reference.
 *
 ****************************************************************************/

static void optee_dummy_ept_release(FAR struct rpmsg_endpoint *ept)
{
  kmm_free(ept);
}

/****************************************************************************
 * Name: optee_dummy_device_created
 *
 * Description:
 *   Announce the channel as soon as an rpmsg device shows up.  Any remote
 *   processor that binds to the channel is served, so no name is matched
 *   here.  The announcement cannot be made from optee_register(), as no
 *   rpmsg device exists yet at that point.
 *
 ****************************************************************************/

static void optee_dummy_device_created(FAR struct rpmsg_device *rdev,
                                       FAR void *priv)
{
  FAR struct rpmsg_endpoint *ept;
  int ret;

  ept = kmm_zalloc(sizeof(*ept));
  if (ept == NULL)
    {
      return;
    }

  ret = rpmsg_create_ept(ept, rdev, OPTEE_DUMMY_EPT_NAME,
                         RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                         optee_dummy_ept_cb, rpmsg_destroy_ept);
  if (ret < 0)
    {
      _err("Failed to announce %s to %s: %d\n",
           OPTEE_DUMMY_EPT_NAME, rpmsg_get_cpuname(rdev), ret);
      kmm_free(ept);
      return;
    }

  /* Installed only once registration has succeeded.  A late failure inside
   * rpmsg_create_ept() drops the reference it took, which would run this
   * callback and free the endpoint that the error path above frees too.
   */

  ept->release_cb = optee_dummy_ept_release;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: optee_register
 *
 * Description:
 *   Announce the channel to whichever remote processor binds to it, so that
 *   the TEE backend running there can reach this side.  Binding hands that
 *   backend an rpmsg device, which is what it needs to allocate out of the
 *   memory shared with this side.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int optee_register(void)
{
  /* rpmsg_register_callback() also runs the callback for a device that is
   * already up, so this works whichever side of rptun initialisation it is
   * called from.
   */

  return rpmsg_register_callback(NULL,
                                 optee_dummy_device_created,
                                 NULL,
                                 NULL,
                                 NULL);
}
