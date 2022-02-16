/****************************************************************************
 * arch/ceva/src/common/up_start.c
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

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/userspace.h>
#include <arch/board/board.h>

#include <string.h>

#include "mpu.h"
#include "up_internal.h"

#if CONFIG_ARCH_NR_MEMORY >= 2
#  define _START_TEXT2  ((const void *)&_stext2)
#  define _END_TEXT2    ((const void *)&_etext2)
#  define _START_BSS2   ((void *)&_sbss2)
#  define _END_BSS2     ((void *)&_ebss2)
#  define _DATA_INIT2   ((const void *)&_eronly2)
#  define _START_DATA2  ((void *)&_sdata2)
#  define _END_DATA2    ((void *)&_edata2)
#else
#  define _START_TEXT2  NULL
#  define _END_TEXT2    NULL
#  define _START_BSS2   NULL
#  define _END_BSS2     NULL
#  define _DATA_INIT2   NULL
#  define _START_DATA2  NULL
#  define _END_DATA2    NULL
#endif

#if CONFIG_ARCH_NR_MEMORY >= 3
#  define _START_TEXT3  ((const void *)&_stext3)
#  define _END_TEXT3    ((const void *)&_etext3)
#  define _START_BSS3   ((void *)&_sbss3)
#  define _END_BSS3     ((void *)&_ebss3)
#  define _DATA_INIT3   ((const void *)&_eronly3)
#  define _START_DATA3  ((void *)&_sdata3)
#  define _END_DATA3    ((void *)&_edata3)
#else
#  define _START_TEXT3  NULL
#  define _END_TEXT3    NULL
#  define _START_BSS3   NULL
#  define _END_BSS3     NULL
#  define _DATA_INIT3   NULL
#  define _START_DATA3  NULL
#  define _END_DATA3    NULL
#endif

#if CONFIG_ARCH_NR_MEMORY >= 4
#  define _START_TEXT4  ((const void *)&_stext4)
#  define _END_TEXT4    ((const void *)&_etext4)
#  define _START_BSS4   ((void *)&_sbss4)
#  define _END_BSS4     ((void *)&_ebss4)
#  define _DATA_INIT4   ((const void *)&_eronly4)
#  define _START_DATA4  ((void *)&_sdata4)
#  define _END_DATA4    ((void *)&_edata4)
#else
#  define _START_TEXT4  NULL
#  define _END_TEXT4    NULL
#  define _START_BSS4   NULL
#  define _END_BSS4     NULL
#  define _DATA_INIT4   NULL
#  define _START_DATA4  NULL
#  define _END_DATA4    NULL
#endif

#if CONFIG_ARCH_NR_MEMORY >= 5
#  error CONFIG_ARCH_NR_MEMORY must between 1 to 4
#endif

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static void init_bss_section(bool priv,
                             void *const bssstart[],
                             void *const bssend[]);

static void init_data_section(bool priv,
                              const void *const datasource[],
                              void *const datastart[],
                              void *const dataend[]);

static void init_text_section(bool priv,
                              const void *const textstart[],
                              const void *const textend[]);

static void init_kernelspace(void);

#ifdef CONFIG_BUILD_PROTECTED
static void init_userspace(void);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void init_bss_section(bool priv,
                             void *const bssstart[],
                             void *const bssend[])
{
  int i;

  for (i = 0; bssstart[i] != _END_MEM || bssend[i] != _END_MEM; i++)
    {
      if (priv)
        {
          mpu_priv_data(bssstart[i], bssend[i] - bssstart[i]);
        }
      else
        {
          mpu_user_data(bssstart[i], bssend[i] - bssstart[i]);
        }

      memset(bssstart[i], 0, bssend[i] - bssstart[i]);
    }
}

static void init_data_section(bool priv,
                              const void *const datasource[],
                              void *const datastart[],
                              void *const dataend[])
{
  int i;

  for (i = 0; datasource[i] != _END_MEM
              || datastart[i] != _END_MEM
              || dataend[i] != _END_MEM; i++)
    {
      if (priv)
        {
          mpu_priv_data(datastart[i], dataend[i] - datastart[i]);
        }
      else
        {
          mpu_user_data(datastart[i], dataend[i] - datastart[i]);
        }

      if (datasource[i] != datastart[i])
        {
          memcpy(datastart[i], datasource[i], dataend[i] - datastart[i]);
        }
    }
}

static void init_text_section(bool priv,
                              const void *const textstart[],
                              const void *const textend[])
{
  int i;

  for (i = 0; textstart[i] != _END_MEM || textend[i] != _END_MEM; i++)
    {
      if (priv)
        {
          mpu_priv_code(textstart[i], textend[i] - textstart[i]);
        }
      else
        {
          mpu_user_code(textstart[i], textend[i] - textstart[i]);
        }
    }
}

static void init_kernelspace(void)
{
  const void *const textstart[] =
  {
    _START_TEXT, _START_TEXT2, _START_TEXT3, _START_TEXT4, _END_MEM,
  };

  const void *const textend[] =
  {
    _END_TEXT, _END_TEXT2, _END_TEXT3, _END_TEXT4, _END_MEM,
  };

  const void *const datasource[] =
  {
    _DATA_INIT, _DATA_INIT2, _DATA_INIT3, _DATA_INIT4, _END_MEM,
  };

  void *const datastart[] =
  {
    _START_DATA, _START_DATA2, _START_DATA3, _START_DATA4, _END_MEM,
  };

  void *const dataend[] =
  {
    _END_DATA, _END_DATA2, _END_DATA3, _END_DATA4, _END_MEM,
  };

  void *const bssstart[] =
  {
    _START_BSS, _START_BSS2, _START_BSS3, _START_BSS4, _END_MEM,
  };

  void *const bssend[] =
  {
    _END_BSS, _END_BSS2, _END_BSS3, _END_BSS4, _END_MEM,
  };

  init_text_section(true, textstart, textend);

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  init_data_section(true, datasource, datastart, dataend);

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  init_bss_section(true, bssstart, bssend);

  /* Initialize the idle stack */

  mpu_priv_data(g_idle_basestack,
    g_idle_topstack - g_idle_basestack);
#ifdef CONFIG_STACK_COLORATION
  up_stack_color(g_idle_basestack,
    g_idle_topstack - g_idle_basestack - 256);
#endif
}

#ifdef CONFIG_BUILD_PROTECTED
static void init_userspace(void)
{
  /* Initialize all of user-space .data */

  DEBUGASSERT(
    USERSPACE->us_datasource != 0 &&
    USERSPACE->us_datastart != 0 &&
    USERSPACE->us_dataend != 0);

  init_text_section(false,
    (const void *const *)USERSPACE->us_textstart,
    (const void *const *)USERSPACE->us_textend);

  init_data_section(false,
    (const void *const *)USERSPACE->us_datasource,
    (void *const *)USERSPACE->us_datastart,
    (void *const *)USERSPACE->us_dataend);

  /* Clear all of user-space .bss */

  DEBUGASSERT(
    USERSPACE->us_bssstart != 0 &&
    USERSPACE->us_bssend != 0);

  init_bss_section(false,
    (void *const *)USERSPACE->us_bssstart,
    (void *const *)USERSPACE->us_bssend);
}
#else
#  define init_userspace()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void up_start(void)
{
  up_enable_icache();
  up_enable_dcache();

  init_kernelspace();
  init_userspace();

  mpu_control(true);

  up_earlyserialinit();
  up_earlyinitialize();
  board_earlyinitialize();

  nx_start();
}
