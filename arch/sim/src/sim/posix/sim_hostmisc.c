/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostmisc.c
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

#include <stdlib.h>

#include "sim_internal.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ARCH_COVERAGE
void __gcov_dump(void);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern uint64_t up_irq_save(void);
extern void up_irq_restore(uint64_t flags);
extern int backtrace(void **array, int size);

/****************************************************************************
 * Name: host_abort
 *
 * Description:
 *   Abort the simulation
 *
 * Input Parameters:
 *   status - Exit status to set
 ****************************************************************************/

void host_abort(int status)
{
  uint64_t flags = up_irq_save();

#ifdef CONFIG_ARCH_COVERAGE
  /* Dump gcov data. */

  __gcov_dump();
#endif

  /* exit the simulation */

  exit(status);

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: host_backtrace
 *
 * Description:
 *   bcaktrace
 *
 * Input Parameters:
 *   array - return array, which backtrace will be stored
 *   size  - array size
 ****************************************************************************/

int host_backtrace(void** array, int size)
{
#ifdef CONFIG_WINDOWS_CYGWIN
  return 0;
#else
  return backtrace(array, size);
#endif
}
