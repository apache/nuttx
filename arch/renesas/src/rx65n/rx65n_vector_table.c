/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_vector_table.c
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

#include "rx65n_macrodriver.h"
#include "chip.h"
#include "stdint.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OFS_REG     locate_data(".ofs1") /* 0xfe7f5d00 */
#define OFS_TMINF   locate_data(".ofs2") /* 0xfe7f5d10 */
#define OFS_SPCC    locate_data(".ofs3") /* 0xfe7f5d40 */
#define OFS_TMEF    locate_data(".ofs4") /* 0xfe7f5d48 */
#define OFS_OSIS    locate_data(".ofs5") /* 0xfe7f5d50 */
#define OFS_FAW     locate_data(".ofs6") /* 0xfe7f5d64 */
#define OFS_ROMCODE locate_data(".ofs7") /* 0xfe7f5d70 */

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
#ifdef __USE_DEBUG_NOP_FOR_BREAKPOINTS
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
#ifdef __USE_DEBUG_NOP_FOR_BREAKPOINTS
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
#ifdef __USE_DEBUG_NOP_FOR_BREAKPOINTS
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
#ifdef __USE_DEBUG_NOP_FOR_BREAKPOINTS
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
#ifdef __USE_DEBUG_NOP_FOR_BREAKPOINTS
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
#ifdef __USE_DEBUG_NOP_FOR_BREAKPOINTS
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
#ifdef __USE_DEBUG_NOP_FOR_BREAKPOINTS
  __asm("nop");
#endif
}

#define EXVECT_SECT    locate_data(".exvectors")

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
    r_floatingpoint_exception, /* 0xffffffe4  Exception */
    r_undefined_exception,     /* 0xffffffe8  Reserved  */
    r_undefined_exception,     /* 0xffffffec  Reserved  */
    r_undefined_exception,     /* 0xfffffff0  Reserved  */
    r_undefined_exception,     /* 0xfffffff4  Reserved  */
    r_nmi_exception            /* 0xfffffff8  NMI       */
};

#define FVECT_SECT    locate_data(".fvectors")
extern void _start(void); /* defined in rx65n_head.S */
const  void *hardware_vectors[] FVECT_SECT =
{
  /* 0xfffffffc  RESET */

  _start      /* Power On Reset PC */
};

#define RVECT_SECT locate_data(".rvectors")
const void *relocatable_vectors[256] RVECT_SECT  =
{
  0
};
