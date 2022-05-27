/****************************************************************************
 * sched/tls/tls.h
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

#ifndef __SCHED_TLS_TLS_H
#define __SCHED_TLS_TLS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sched.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tls_init_info
 *
 * Description:
 *   Allocate and initilize tls_info_s structure.
 *
 * Input Parameters:
 *   - tcb: The TCB of new task
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tls_init_info(FAR struct tcb_s *tcb);

/****************************************************************************
 * Name: tls_dup_info
 *
 * Description:
 *   Allocate and duplicate tls_info_s structure.
 *
 * Input Parameters:
 *   - dst: The TCB of new task
 *   - src: The TCB of source task
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tls_dup_info(FAR struct tcb_s *dst, FAR struct tcb_s *src);

#endif /* __SCHED_TLS_TLS_H */
