/****************************************************************************
 * include/nuttx/ioexpander/ioe_dummy.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_IOE_DUMMY_H
#define __INCLUDE_NUTTX_IOEXPANDER_IOE_DUMMY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/ioexpander/ioexpander.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_DUMMY
FAR struct ioexpander_dev_s *ioe_dummy_initialize(void);
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_IOE_DUMMY_H */
