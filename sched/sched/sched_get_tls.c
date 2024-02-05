/****************************************************************************
 * sched/sched/sched_get_tls.c
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

#include "nuttx/sched.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_tls
 *
 * Description:
 *   Get TLS of any task / tcb with no security checks.
 *
 * Input Parameters:
 *   tcb - The tcb to query.
 *
 * Returned Value:
 *   Pointer to the TLS structure.
 *
 ****************************************************************************/

FAR struct tls_info_s *nxsched_get_tls(FAR struct tcb_s *tcb)
{
  /* The TLS data lies at the lowest address of the stack allocation.
   * This is true for both push-up and push-down stacks.
   */

  return (FAR struct tls_info_s *)tcb->stack_alloc_ptr;
}
