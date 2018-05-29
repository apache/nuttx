/****************************************************************************
 * libs/libc/hex2bin/hex2mem.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <assert.h>
#include <hex2bin.h>

#include <nuttx/streams.h>

#ifdef CONFIG_LIB_HEX2BIN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hex2mem
 *
 * Description:
 *   Read the Intel HEX ASCII data provided on the file descriptor 'fd' and
 *   write the binary to memory.
 *
 *   If, for example, fd is zero (stdin), then the HEX ASCII data would be
 *   taken from the console and written to memory.
 *
 * Input Parameters:
 *   fd        - The file descriptor from which Intel HEX data will be
 *               received.
 *   baseaddr  - The base address of the memory region stream.
 *   endpaddr  - The end address (plus 1) of the memory region.
 *   swap      - Controls byte ordering.  See enum hex2bin_swap_e for
 *               description of the values.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int hex2mem(int fd, uint32_t baseaddr, uint32_t endpaddr,
            enum hex2bin_swap_e swap)
{
  struct lib_rawinstream_s rawinstream;
  struct lib_memsostream_s memoutstream;

  /* Check memory addresses */

  DEBUGASSERT(fd >= 0 && endpaddr > baseaddr);

  /* Wrap the file descriptor as raw stream; wrap the memory as a memory
   * stream.
   */

  lib_rawinstream(&rawinstream, fd);
  lib_memsostream(&memoutstream, (FAR char *)baseaddr,
                  (int)(endpaddr - baseaddr));

  /* And do the deed */

  return hex2bin(&rawinstream.public, &memoutstream.public,
                 (uint32_t)baseaddr, (uint32_t)endpaddr,
                 (enum hex2bin_swap_e)swap);
}

#endif /* CONFIG_LIB_HEX2BIN */
