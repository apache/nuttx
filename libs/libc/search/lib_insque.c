/****************************************************************************
 * libs/libc/search/lib_insque.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <search.h>
#include <stddef.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The elements of insque() and remque() start with these two pointers */

struct qelem_s
{
  FAR struct qelem_s *q_forw;
  FAR struct qelem_s *q_back;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: insque
 *
 * Description:
 *   Insert element into a doubly linked queue right after pred.  If pred is
 *   NULL, element starts a new linear queue: both its pointers become NULL.
 *
 ****************************************************************************/

void insque(FAR void *element, FAR void *pred)
{
  FAR struct qelem_s *e = element;
  FAR struct qelem_s *p = pred;

  if (p == NULL)
    {
      e->q_forw = NULL;
      e->q_back = NULL;
      return;
    }

  e->q_forw = p->q_forw;
  e->q_back = p;
  if (p->q_forw != NULL)
    {
      p->q_forw->q_back = e;
    }

  p->q_forw = e;
}

/****************************************************************************
 * Name: remque
 *
 * Description:
 *   Remove element from its doubly linked queue.
 *
 ****************************************************************************/

void remque(FAR void *element)
{
  FAR struct qelem_s *e = element;

  if (e->q_forw != NULL)
    {
      e->q_forw->q_back = e->q_back;
    }

  if (e->q_back != NULL)
    {
      e->q_back->q_forw = e->q_forw;
    }
}
