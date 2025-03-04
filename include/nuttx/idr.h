/****************************************************************************
 * include/nuttx/idr.h
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

#ifndef __INCLUDE_NUTTX_IDR_H
#define __INCLUDE_NUTTX_IDR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define idr_for_each_entry(idr, node, id) \
  for ((id) = 0; ((node) = idr_get_next(idr, &(id))) != NULL; id++)

#define idr_init() idr_init_base(0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct idr_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR void *idr_find(FAR struct idr_s *idr, unsigned int id);
FAR void *idr_get_next(FAR struct idr_s *idr, FAR int *id);
FAR struct idr_s *idr_init_base(int base);
void idr_destroy(FAR struct idr_s *idr);
int idr_alloc_u32(FAR struct idr_s *idr, FAR void *ptr, FAR uint32_t *result,
                  uint32_t max);
int idr_alloc(FAR struct idr_s *idr, FAR void *ptr, int start, int end);
FAR void *idr_remove(FAR struct idr_s *idr, unsigned int id);

#endif
