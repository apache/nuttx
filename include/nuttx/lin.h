/****************************************************************************
 * include/nuttx/lin.h
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

#ifndef __INCLUDE_NUTTX_LIN_H
#define __INCLUDE_NUTTX_LIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/can.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LIN_ID_BITS             6

#define LIN_ID_MASK             ((1 << LIN_ID_BITS) - 1)
#define LIN_ID_MAX              LIN_ID_MASK

#define LIN_CACHE_RESPONSE      (1 << (LIN_ID_BITS))
#define LIN_CHECKSUM_EXTENDED   (1 << (LIN_ID_BITS + 1))
#define LIN_SINGLE_RESPONSE     (1 << (LIN_ID_BITS + 2))

#endif /* __INCLUDE_NUTTX_LIN_H */
