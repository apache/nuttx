/****************************************************************************
 * libs/libc/tls/tls_getvalue.c
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
 * Name: tls_get_value
 *
 * Description:
 *   Return an the TLS data value associated with the 'tlsindx'
 *
 * Input Parameters:
 *   tlsindex - Index of TLS data element to return
 *
 * Returned Value:
 *   The value of TLS element associated with 'tlsindex'. Errors are not
 *   reported.  Zero is returned in the event of an error, but zero may also
 *   be valid value and returned when there is no error.  The only possible
 *   error would be if tlsindex < 0 or tlsindex >=CONFIG_TLS_NELEM.
 *
 ****************************************************************************/

uintptr_t tls_get_value(int tlsindex)
{
  FAR struct tls_info_s *info;
  uintptr_t ret = 0;

  DEBUGASSERT(tlsindex >= 0 && tlsindex < CONFIG_TLS_NELEM);
  if (tlsindex >= 0 && tlsindex < CONFIG_TLS_NELEM)
    {
      /* Get the TLS info structure from the current threads stack */

      info = up_tls_info();
      DEBUGASSERT(info != NULL);

      /* Get the element value from the TLS info. */

      ret = info->tl_elem[tlsindex];
    }

  return ret;
}

#endif /* CONFIG_TLS_NELEM > 0 */
