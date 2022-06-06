/****************************************************************************
 * libs/libc/tls/tls_getinfo.c
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

#if !defined(up_tls_info) && (defined(__KERNEL__) || !defined(CONFIG_TLS_ALIGNED))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_get_info
 *
 * Description:
 *   Return a reference to the tls_info_s structure.  This is used as part
 *   of the internal implementation of tls_get/set_elem() and ONLY for the
 *   where CONFIG_TLS_ALIGNED is *not* defined or __KERNEL__ is defined.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the thread-specific tls_info_s structure is return on
 *   success.  NULL would be returned in the event of any failure.
 *
 ****************************************************************************/

FAR struct tls_info_s *tls_get_info(void)
{
  FAR struct tls_info_s *info = NULL;
  struct stackinfo_s stackinfo;
  int ret;

  ret = nxsched_get_stackinfo(0, &stackinfo);
  if (ret >= 0)
    {
      /* The TLS data lies at the lowest address of the stack allocation.
       * This is true for both push-up and push-down stacks.
       */

      info = (FAR struct tls_info_s *)stackinfo.stack_alloc_ptr;
    }

  return info;
}

#endif /* !defined(up_tls_info) && (defined(__KERNEL__) || !defined(CONFIG_TLS_ALIGNED)) */
