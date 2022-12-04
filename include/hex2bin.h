/****************************************************************************
 * include/hex2bin.h
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

#ifndef __INCLUDE_HEX2BIN_H
#define __INCLUDE_HEX2BIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>

#ifdef CONFIG_LIBC_HEX2BIN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Intel HEX data streams are normally in big endian order.  The following
 * enumeration selects other ordering.
 */

enum hex2bin_swap_e
{
  HEX2BIN_NOSWAP = 0, /* No swap, stream is in the correct byte order */
  HEX2BIN_SWAP16 = 1, /* Swap bytes in 16-bit values */
  HEX2BIN_SWAP32 = 2  /* Swap bytes in 32-bit values */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hex2bin
 *
 * Description:
 *   Read the Intel HEX ASCII data provided on the serial IN stream and write
 *   the binary to the seek-able serial OUT stream.
 *
 *   These streams may be files or, in another usage example, the IN stream
 *   could be a serial port and the OUT stream could be a memory stream.
 *   This would decode and write the serial input to memory.
 *
 * Input Parameters:
 *   instream  - The incoming stream from which Intel HEX data will be
 *               received.
 *   outstream - The outgoing stream in which binary data will be written.
 *   baseaddr  - The base address of the outgoing stream.  Seeking in the
 *               output stream will be relative to this address.
 *   endpaddr  - The end address (plus 1) of the outgoing stream.  This
 *               value is used only for range checking.  endpaddr must
 *               be larger than baseaddr.  A zero value for endpaddr
 *               disables range checking.
 *   swap      - Controls byte ordering.  See enum hex2bin_swap_e for
 *               description of the values.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

struct lib_instream_s;
struct lib_sostream_s;
int hex2bin(FAR struct lib_instream_s *instream,
            FAR struct lib_sostream_s *outstream, uint32_t baseaddr,
            uint32_t endpaddr, enum hex2bin_swap_e swap);

/****************************************************************************
 * Name: hex2mem
 *
 * Description:
 *   Read the Intel HEX ASCII data provided on the file descriptor 'fd' and
 *   write the binary to memory.
 *
 *   If, for example, fd is zero (corresponding to stdin), then the HEX
 *   ASCII data would be taken from the console and written to memory.
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
            enum hex2bin_swap_e swap);

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
             enum hex2bin_swap_e swap);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SYSTEM_HEX2BIN */
#endif /* __INCLUDE_HEX2BIN_H */
