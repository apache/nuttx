/****************************************************************************
 * arch/x86_64/src/intel64/intel64_restore_auxstate.c
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

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include <arch/arch.h>
#include <arch/irq.h>
#include <arch/io.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_restore_auxstate
 *
 * Description:
 *   This function performs some additional action required to complete the
 *   CTX on intel64 processor.
 *
 ****************************************************************************/

void x86_64_restore_auxstate(struct tcb_s *rtcb)
{
  /* Set PCID, avoid TLB flush */

  set_pcid(rtcb->pid);
}
