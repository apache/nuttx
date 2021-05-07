/****************************************************************************
 * libs/libnx/nxglib/nxglib_trapcopy.c
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

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_trapcopy
 *
 * Description:
 *   This is essentially memcpy for trapezoids.  We don't do structure
 *   assignments because some compilers are not good at that.
 *
 ****************************************************************************/

void nxgl_trapcopy(FAR struct nxgl_trapezoid_s *dest,
                   FAR const struct nxgl_trapezoid_s *src)
{
  nxgl_runcopy(&dest->top, &src->top);
  nxgl_runcopy(&dest->bot, &src->bot);
}
