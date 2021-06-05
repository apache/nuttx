/****************************************************************************
 * libs/libc/tls/tls_getdtor.c
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
#include <nuttx/spinlock.h>
#include <nuttx/tls.h>
#include <arch/tls.h>

#if CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_get_dtor
 *
 * Description:
 *   Get the TLS element destructor associated with the 'tlsindex' to 'dtor'
 *
 * Input Parameters:
 *   tlsindex - Index of TLS data destructor to get
 *
 * Returned Value:
 *   A non-null destruct function pointer.
 *
 ****************************************************************************/

tls_dtor_t tls_get_dtor(int tlsindex)
{
  FAR struct task_info_s *tinfo = task_get_info();
  tls_dtor_t dtor;

  DEBUGASSERT(tinfo != NULL);
  DEBUGASSERT(tlsindex >= 0 && tlsindex < CONFIG_TLS_NELEM);

  dtor = tinfo->ta_tlsdtor[tlsindex];

  return dtor;
}

#endif /* CONFIG_TLS_NELEM > 0 */
