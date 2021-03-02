/****************************************************************************
 * libs/libc/sched/sched_cpucount.c
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

#include <sched.h>

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_cpucount
 *
 * Description:
 *   Return the number of bits set in the 'set'.  This could be improved by
 *   using CPU-specific bit counting instructions.
 *
 * Input Parameters:
 *   set - The set of CPUs to be counted.
 *
 * Returned Value:
 *   The number of CPUs in 'set'
 *
 ****************************************************************************/

int sched_cpucount(FAR const cpu_set_t *set)
{
  int count;
  int cpu;

  for (cpu = 0, count = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      if ((*set & (1 << cpu)) != 0)
        {
          count++;
        }
    }

  return count;
}

#endif /* CONFIG_SMP */
