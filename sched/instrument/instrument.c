/****************************************************************************
 * sched/instrument/instrument.c
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

#include <nuttx/instrument.h>

/****************************************************************************
 * External Definitions
 ****************************************************************************/

#if CONFIG_SCHED_STACK_RECORD > 0
extern struct instrument_s g_stack_record;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: instrument_initialize
 *
 * Description:
 *   This function is called to initialize all the instrument functions
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

void instrument_initialize(void)
{
#if CONFIG_SCHED_STACK_RECORD > 0
  instrument_register(&g_stack_record);
#endif
}
