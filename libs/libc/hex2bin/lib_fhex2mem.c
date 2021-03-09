/****************************************************************************
 * libs/libc/hex2bin/lib_fhex2mem.c
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

#include <stdio.h>
#include <assert.h>
#include <hex2bin.h>

#include <nuttx/streams.h>

#ifdef CONFIG_LIB_HEX2BIN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fhex2mem
 *
 * Description:
 *   Read the Intel HEX ASCII data provided on the standard stream
 *   'instream' and write the binary to memory.
 *
 *   If, for example, instream is stdin, then the HEX ASCII data would be
 *   taken from the console and written to memory.
 *
 * Input Parameters:
 *   instream  - The incoming standard stream from which Intel HEX data
 *               will be received.
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

int fhex2mem(FAR FILE *instream, uint32_t baseaddr, uint32_t endpaddr,
             enum hex2bin_swap_e swap)
{
  struct lib_stdinstream_s stdinstream;
  struct lib_memsostream_s memoutstream;

  /* Check memory addresses */

  DEBUGASSERT(instream != NULL && endpaddr > baseaddr);

  /* Wrap the file descriptor as raw stream; wrap the memory as a memory
   * stream.
   */

  lib_stdinstream(&stdinstream, instream);
  lib_memsostream(&memoutstream, (FAR char *)baseaddr,
                  (int)(endpaddr - baseaddr));

  /* And do the deed */

  return hex2bin(&stdinstream.public, &memoutstream.public,
                 (uint32_t)baseaddr, (uint32_t)endpaddr,
                 (enum hex2bin_swap_e)swap);
}

#endif /* CONFIG_LIB_HEX2BIN */
