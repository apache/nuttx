/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_vector_table.c
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
 ***************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "rx65n_macrodriver.h"
#include "chip.h"
#include "stdint.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OFS_REG     __attribute__ ((section (".ofs1"))) /* 0xfe7f5d00 */
#define OFS_TMINF   __attribute__ ((section (".ofs2"))) /* 0xfe7f5d10 */
#define OFS_SPCC    __attribute__ ((section (".ofs3"))) /* 0xfe7f5d40 */
#define OFS_TMEF    __attribute__ ((section (".ofs4"))) /* 0xfe7f5d48 */
#define OFS_OSIS    __attribute__ ((section (".ofs5"))) /* 0xfe7f5d50 */
#define OFS_FAW     __attribute__ ((section (".ofs6"))) /* 0xfe7f5d64 */
#define OFS_ROMCODE __attribute__ ((section (".ofs7"))) /* 0xfe7f5d70 */

/* SPCC register */

const unsigned long     __spccreg    OFS_SPCC = 0xffffffff;

/* TMEF register */

const unsigned long     __tmefreg    OFS_TMEF = 0xffffffff;

/* OSIS register (ID codes) */

const unsigned long     __osisreg[4] OFS_OSIS =
{
  0xffffffff,
  0xffffffff,
  0xffffffff,
  0xffffffff
};

/* TMINF register */

const unsigned long     __tminfreg    OFS_TMINF   = 0xffffffff;

/* FAW register */

const unsigned long     __fawreg      OFS_FAW     = 0xffffffff;

/* ROMCODE register */

const unsigned long     __romcodereg  OFS_ROMCODE = 0xffffffff;

/* MDE register (Single Chip Mode) */

#ifdef __RX_BIG_ENDIAN__
const unsigned long  __mdereg   OFS_REG = 0xfffffff8; /* big */
#else
const unsigned long  __mdereg   OFS_REG = 0xffffffff; /* little */
#endif

const unsigned long __ofs0reg OFS_REG = 0xffffffff; /* OFS0 register */
const unsigned long __ofs1reg OFS_REG = 0xffffffff; /* OFS1 register */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: r_undefined_exception
 *
 * Description:
 * Handler for undefined exception
 ****************************************************************************/

void r_undefined_exception(void)
{
#ifdef  __USE_DEBUG_NOP_FOR_BREAKPOINTS
  __asm("nop");
#endif
}

/****************************************************************************
 * Name: r_reserved_exception
 *
 * Description:
 * Handler for reserved exception
 ****************************************************************************/

void r_reserved_exception(void)
{
#ifdef  __USE_DEBUG_NOP_FOR_BREAKPOINTS
  __asm("nop");
#endif
}

/****************************************************************************
 * Name: r_nmi_exception
 *
 * Description:
 * Handler for nmi exception
 ****************************************************************************/

void r_nmi_exception(void)
{
#ifdef  __USE_DEBUG_NOP_FOR_BREAKPOINTS
  __asm("nop");
#endif
}

/****************************************************************************
 * Name: r_brk_exception
 *
 * Description:
 * Handler for brk exception
 ****************************************************************************/

void r_brk_exception(void)
{
#ifdef  __USE_DEBUG_NOP_FOR_BREAKPOINTS
  __asm("nop");
#endif
}

/****************************************************************************
 * Name: r_privileged_exception
 *
 * Description:
 * Handler for privileged  exception
 ****************************************************************************/

void r_privileged_exception(void)
{
#ifdef  __USE_DEBUG_NOP_FOR_BREAKPOINTS
  __asm("nop");
#endif
}

/****************************************************************************
 * Name: r_access_exception
 *
 * Description:
 * Handler for access  exception
 ****************************************************************************/

void r_access_exception(void)
{
#ifdef  __USE_DEBUG_NOP_FOR_BREAKPOINTS
  __asm("nop");
#endif
}
/****************************************************************************
 * Name: r_floatingpoint_exception
 *
 * Description:
 * Handler for floating point  exception
 ****************************************************************************/

void r_floatingpoint_exception(void)
{
#ifdef  __USE_DEBUG_NOP_FOR_BREAKPOINTS
  __asm("nop");
#endif
}

#define EXVECT_SECT    __attribute__ ((section (".exvectors")))

const void *except_vectors[] EXVECT_SECT  =
{
    r_reserved_exception,      /* 0xffffff80  Reserved  */
    r_reserved_exception,      /* 0xffffff84  Reserved  */
    r_reserved_exception,      /* 0xffffff88  Reserved  */
    r_reserved_exception,      /* 0xffffff8c  Reserved  */
    r_reserved_exception,      /* 0xffffff90  Reserved  */
    r_reserved_exception,      /* 0xffffff94  Reserved  */
    r_reserved_exception,      /* 0xffffff98  Reserved  */
    r_reserved_exception,      /* 0xffffff9c  Reserved  */
    r_reserved_exception,      /* 0xffffffa0  Reserved  */
    r_reserved_exception,      /* 0xffffffa4  Reserved  */
    r_reserved_exception,      /* 0xffffffa8  Reserved  */
    r_reserved_exception,      /* 0xffffffac  Reserved  */
    r_reserved_exception,      /* 0xffffffb0  Reserved  */
    r_reserved_exception,      /* 0xffffffb4  Reserved  */
    r_reserved_exception,      /* 0xffffffb8  Reserved  */
    r_reserved_exception,      /* 0xffffffbc  Reserved  */
    r_reserved_exception,      /* 0xffffffc0  Reserved  */
    r_reserved_exception,      /* 0xffffffc4  Reserved  */
    r_reserved_exception,      /* 0xffffffc8  Reserved  */
    r_reserved_exception,      /* 0xffffffcc  Reserved  */
    r_privileged_exception,    /* 0xffffffd0  Exception */
    r_access_exception,        /* 0xffffffd4  Exception */
    r_reserved_exception,      /* 0xffffffd8  Reserved  */
    r_undefined_exception,     /* 0xffffffdc  Exception */
    r_reserved_exception,      /* 0xffffffe0  Reserved  */
    r_floatingpoint_exception,    /* 0xffffffe4  Exception */
    r_undefined_exception,     /* 0xffffffe8  Reserved  */
    r_undefined_exception,     /* 0xffffffec  Reserved  */
    r_undefined_exception,     /* 0xfffffff0  Reserved  */
    r_undefined_exception,     /* 0xfffffff4  Reserved  */
    r_nmi_exception            /* 0xfffffff8  NMI       */
};

#define FVECT_SECT    __attribute__ ((section (".fvectors")))
extern void _start(void); /* defined in rx65n_head.S */
const  void *hardware_vectors[] FVECT_SECT =
{
  /* 0xfffffffc  RESET */

  _start      /* Power On Reset PC */
};

#define RVECT_SECT __attribute__ ((section (".rvectors")))
const void *relocatable_vectors[256] RVECT_SECT  =
{
  0
};
