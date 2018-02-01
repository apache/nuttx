/****************************************************************************
 * include/hex2bin.h
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

#ifndef __INCLUDE_HEX2BIN_H
#define __INCLUDE_HEX2BIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_LIB_HEX2BIN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some environments may return CR as end-of-line, others LF, and others
 * both.  If not specified, the logic here assumes either (but not both) as
 * the default.
 */

#if defined(CONFIG_EOL_IS_CR)
#  undef  CONFIG_EOL_IS_LF
#  undef  CONFIG_EOL_IS_BOTH_CRLF
#  undef  CONFIG_EOL_IS_EITHER_CRLF
#elif defined(CONFIG_EOL_IS_LF)
#  undef  CONFIG_EOL_IS_CR
#  undef  CONFIG_EOL_IS_BOTH_CRLF
#  undef  CONFIG_EOL_IS_EITHER_CRLF
#elif defined(CONFIG_EOL_IS_BOTH_CRLF)
#  undef  CONFIG_EOL_IS_CR
#  undef  CONFIG_EOL_IS_LF
#  undef  CONFIG_EOL_IS_EITHER_CRLF
#elif defined(CONFIG_EOL_IS_EITHER_CRLF)
#  undef  CONFIG_EOL_IS_CR
#  undef  CONFIG_EOL_IS_LF
#  undef  CONFIG_EOL_IS_BOTH_CRLF
#else
#  undef  CONFIG_EOL_IS_CR
#  undef  CONFIG_EOL_IS_LF
#  undef  CONFIG_EOL_IS_BOTH_CRLF
#  define CONFIG_EOL_IS_EITHER_CRLF 1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Intel HEX data steams are normally in big endian order.  The following
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
 *   could be a serial port and the OUT stream could be a memory stream.  This
 *   would decode and write the serial input to memory.
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

/****************************************************************************
 * Name: hex2bin_main
 *
 * Description:
 *   Main entry point when hex2bin is built as an NSH built-in task.
 *
 * Input Parameters:
 *   Standard task inputs
 *
 * Returned Value:
 *   EXIT_SUCESS on success; EXIT_FAILURE on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_HEX2BIN_BUILTIN
int hex2bin_main(int argc, char **argv);
#endif /* CONFIG_SYSTEM_HEX2BIN_BUILTIN */

/****************************************************************************
 * Name: hex2mem_main
 *
 * Description:
 *   Main entry point when hex2mem is built as an NSH built-in task.
 *
 * Input Parameters:
 *   Standard task inputs
 *
 * Returned Value:
 *   EXIT_SUCESS on success; EXIT_FAILURE on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_HEX2MEM_BUILTIN
int hex2mem_main(int argc, char **argv);
#endif /* CONFIG_SYSTEM_HEX2MEM_BUILTIN */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SYSTEM_HEX2BIN */
#endif /* __INCLUDE_HEX2BIN_H */
