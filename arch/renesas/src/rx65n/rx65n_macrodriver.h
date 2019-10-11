/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_macrodriver.h
 *
 *   Copyright (C) 2008-2019 Gregory Nutt. All rights reserved.
 *   Author: Anjana <anjana@tataelxsi.co.in>
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

#ifndef __ARCH_RENESAS_SRC_RX65N_STATUS_H
#define __ARCH_RENESAS_SRC_RX65N_STATUS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#include "rx65n/iodefine.h"

#ifndef TRUE
#define TRUE (1)
#else
#if (1 != TRUE)
#error "TRUE is not defined by 1."
#endif
#endif

#ifndef FALSE
#define FALSE (0)
#else
#if (0 != FALSE)
#error "FALSE is not defined by 0."
#endif
#endif

#ifndef __TYPEDEF__

/* Status list definition */

#define MD_STATUSBASE      (0x00U)
#define MD_OK              (MD_STATUSBASE + 0x00U) /* register setting OK */
#define MD_SPT             (MD_STATUSBASE + 0x01U) /* IIC stop */
#define MD_NACK            (MD_STATUSBASE + 0x02U) /* IIC no ACK */
#define MD_BUSY1           (MD_STATUSBASE + 0x03U) /* busy 1 */
#define MD_BUSY2           (MD_STATUSBASE + 0x04U) /* busy 2 */

/* Error list definition */

#define MD_ERRORBASE   (0x80U)
#define MD_ERROR       (MD_ERRORBASE + 0x00U)  /* error */

/* error argument input error */

#define MD_ARGERROR    (MD_ERRORBASE + 0x01U)
#define MD_ERROR1      (MD_ERRORBASE + 0x02U)  /* error 1 */
#define MD_ERROR2      (MD_ERRORBASE + 0x03U)  /* error 2 */
#define MD_ERROR3      (MD_ERRORBASE + 0x04U)  /* error 3 */
#define MD_ERROR4      (MD_ERRORBASE + 0x05U)  /* error 4 */
#define MD_ERROR5      (MD_ERRORBASE + 0x06U)  /* error 5 */

#define nop()              asm("nop;")
#define brk()              asm("brk;")
#define wait()             asm("wait;")
#endif

#ifndef __TYPEDEF__
#ifndef _STDINT_H
  typedef unsigned char  uint8_t;
  typedef unsigned short uint16_t;
#endif
  typedef unsigned short MD_STATUS;
#define __TYPEDEF__
#endif

#endif /* __ARCH_RENESAS_SRC_RX65N_STATUS_H */
