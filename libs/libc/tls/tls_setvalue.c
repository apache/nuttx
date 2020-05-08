/****************************************************************************
 * libs/libc/tls/tls_setvalue.c
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

#include <stdint.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/tls.h>
#include <arch/tls.h>

#if CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_set_value
 *
 * Description:
 *   Set the TLS element associated with the 'tlsindex' to 'tlsvalue'
 *
 * Input Parameters:
 *   tlsindex - Index of TLS data element to set
 *   tlsvalue - The new value of the TLS data element
 *
 * Returned Value:
 *   Zero is returned on success, a negated errno value is return on
 *   failure:
 *
 *     EINVAL - tlsindex is not in range.
 *
 ****************************************************************************/

int tls_set_value(int tlsindex, uintptr_t tlsvalue)
{
  FAR struct tls_info_s *info;

  DEBUGASSERT(tlsindex >= 0 && tlsindex < CONFIG_TLS_NELEM);
  if (tlsindex >= 0 && tlsindex < CONFIG_TLS_NELEM)
    {
      /* Get the TLS info structure from the current threads stack */

      info = up_tls_info();
      DEBUGASSERT(info != NULL);

      /* Set the element value int the TLS info. */

      info->tl_elem[tlsindex] = tlsvalue;
      return OK;
    }

  return -EINVAL;
}

#endif /* CONFIG_TLS_NELEM > 0 */
