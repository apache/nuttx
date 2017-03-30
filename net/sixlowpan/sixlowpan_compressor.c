/****************************************************************************
 * net/sixlowpan/sixlowpan_compressor.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include "nuttx/net/net.h"
#include "nuttx/net/sixlowpan.h"

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sixlowpan_set_compressor
 *
 * Description:
 *   Configure to use the architecture-specific compressor.
 *
 * Input parameters:
 *   compressor - A reference to the new compressor to be used.  This may
 *                be a NULL value to disable the compressor.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_set_compressor(FAR struct sixlowpan_nhcompressor_s *compressor)
{
  /* Make sure that the compressor is not in use */

  net_lock();

  /* Then instantiate the new compressor */

  g_sixlowpan_compressor = compressor;
  net_unlock();
}

#endif /* CONFIG_NET_6LOWPAN */
