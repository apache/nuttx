/****************************************************************************
 * libs/libc/misc/lib_uadd64.c
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

#include <nuttx/lib/math32.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uadd64
 *
 * Description:
 *   Add two 64-bit values and return a 64-bit sum.
 *
 * Input Parameters:
 *   term1 and term2 - The values to be added
 *   sum - The location to return the product of the two values.  sum may
 *     be one of term1 or term2
 *
 ****************************************************************************/

void uadd64(FAR const struct uint64_s *term1,
            FAR const struct uint64_s *term2,
            FAR struct uint64_s *sum)
{
  /* Get the MS part of the sum */

  sum->ms = term1->ms + term2->ms;

  /* Check for carry, i.e., that is when:
   *
   * term1->ls + term2->ls > UINT32_MAX
   */

  if ((UINT32_MAX - term1->ls) > term2->ls)
    {
      sum->ms++;
    }

  /* Get the LS part of the sum */

  sum->ls = term1->ls + term2->ls;
}
