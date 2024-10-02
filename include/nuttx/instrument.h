/****************************************************************************
 * include/nuttx/instrument.h
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

#ifndef __INCLUDE_NUTTX_INSTRUMENT_H
#define __INCLUDE_NUTTX_INSTRUMENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/queue.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE void (instrument_fun_t)(FAR void *this_fn,
                                     FAR void *call_site,
                                     FAR void *arg);

struct instrument_s
{
  sq_entry_t entry;
  FAR instrument_fun_t *enter;
  FAR instrument_fun_t *leave;
  FAR void *arg;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: instrument_register
 *
 * Description: register instrument, it will be called
 *              when function enter or exit.
 *
 * Input Parameters:
 *   entry - instrument entry structure.
 * Notice:
 *  use CONFIG_ARCH_INSTRUMENT_ALL must mark _start or entry
 *  noinstrument_function, becuase bss not set.
 *  Make sure your callbacks are not instrumented recursively.
 *
 ****************************************************************************/

void instrument_register(FAR struct instrument_s *entry);

#endif /* __INCLUDE_NUTTX_INSTRUMENT_H */
