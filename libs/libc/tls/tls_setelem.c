/****************************************************************************
 * libs/libc/fixedmath/tls_setelem.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#ifdef CONFIG_TLS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_get_element
 *
 * Description:
 *   Set the TLS element associated with the 'elem' index to 'value'
 *
 * Input Parameters:
 *   elem  - Index of TLS element to set
 *   value - The new value of the TLS element
 *
 * Returned Value:
 *   None.  Errors are not reported.  The only possible error would be if
 *   elem >=CONFIG_TLS_NELEM.
 *
 ****************************************************************************/

void tls_set_element(int elem, uintptr_t value)
{
  FAR struct tls_info_s *info;

  DEBUGASSERT(elem >= 0 && elem < CONFIG_TLS_NELEM);
  if (elem >= 0 && elem < CONFIG_TLS_NELEM)
    {
      /* Get the TLS info structure from the current threads stack */

      info = up_tls_info();
      DEBUGASSERT(info != NULL);

      /* Set the element value int the TLS info. */

      info->tl_elem[elem] = value;
    }
}

#endif /* CONFIG_TLS */
