/****************************************************************************
 * graphics/nxmu/nxmu_releasebkgd.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include "nxmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_releasebkgd
 *
 * Description:
 *   Release the background window previously acquired using
 *   nxmu_openbgwindow and return control of the background to NX.
 *
 * Input Parameters:
 *   nxmu - The NXMU state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmu_releasebkgd(FAR struct nxmu_state_s *nxmu)
{
  FAR struct nxbe_state_s *be;

  DEBUGASSERT(nxmu != NULL);

  /* Destroy the client window callbacks and restore the server connection. */

  be            = &nxmu->be;
  be->bkgd.cb   = NULL;
  be->bkgd.arg  = NULL;
  be->bkgd.conn = &nxmu->conn;

  /* Redraw the background window */

  nxmu_redraw(&be->bkgd, &be->bkgd.bounds);
}
