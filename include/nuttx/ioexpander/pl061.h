/****************************************************************************
 * include/nuttx/ioexpander/pl061.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_PL061_H
#define __INCLUDE_NUTTX_IOEXPANDER_PL061_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pl061_ioe_initialize
 *
 * Description:
 *   Initialize pl061 device.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *pl061_ioe_initialize(uintptr_t base, int irq);

#endif  //__INCLUDE_NUTTX_IOEXPANDER_PL061_H
