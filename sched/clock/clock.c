/****************************************************************************
 * sched/clock/clock.c
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

#include <time.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  clock
 *
 * Description:
 *   The clock() function returns the implementation's best approximation to
 *   the processor time used by the process since the beginning of a
 *   implementation-defined era related only to the process invocation.
 *
 *   To determine the time in seconds, the value returned by clock() should
 *   be divided by the value of the macro CLOCKS_PER_SEC as defined in
 *   <time.h>.
 *
 *   NOTE:  The current implementation does NOT return the processor time
 *   used by the process.  NuttX currently does not keep any record of
 *   processing time.  Instead the elapsed time since power up is returned.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The system time in units of clock ticks is returned.  If the processor
 *   time used is not available or its value cannot be represented, the
 *   function will return the value (clock_t)-1.
 *
 ****************************************************************************/

clock_t clock(void)
{
  return (clock_t)clock_systime_ticks();
}
