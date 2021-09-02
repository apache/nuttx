/****************************************************************************
 * arch/sim/src/sim/up_macho_init.c
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

#include <sys/mman.h>

#include <assert.h>
#include <stdlib.h>
#include <unistd.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

typedef void (*init_func_t)(int argc, const char *argv[],
                            const char *envp[], const char *apple[]);
extern init_func_t mod_init_func_start \
__asm("section$start$__DATA$__mod_init_func");
extern init_func_t mod_init_func_end \
__asm("section$end$__DATA$__mod_init_func");

static void noop(int argc, const char *argv[], const char *envp[],
                 const char *apple[])
{
  /* nothing */
}

static init_func_t *g_saved_init_funcs;
static unsigned int g_num_saved_init_funcs;
static int g_saved_argc;
static const char **g_saved_argv;
static const char **g_saved_envp;
static const char **g_saved_apple;

static void
allow_write(const void *start, const void *end)
{
  const size_t page_size = sysconf(_SC_PAGE_SIZE);
  const size_t page_mask = ~(page_size - 1);
  void *p = (void *)((uintptr_t)start & page_mask);
  size_t sz = ((uintptr_t)end - (uintptr_t)p + page_size - 1) & ~page_mask;

  /* It seems that Monterey (12.1) maps the section read-only.
   * Make it writable as we want to patch it.
   * This was not necessary for Mojave.
   * Ignore failures as this might not be critical, depending on
   * the OS version.
   */

  mprotect(p, sz, PROT_READ | PROT_WRITE);
}

__attribute__((constructor))
static void save_and_replace_init_funcs(int argc, const char *argv[],
                                        const char *envp[],
                                        const char *apple[])
{
  init_func_t *fp;
  unsigned int nfuncs = &mod_init_func_end - &mod_init_func_start;

  assert(nfuncs > 0);
  g_num_saved_init_funcs = nfuncs - 1;
  if (g_num_saved_init_funcs == 0)
    {
      /* This function is the only constructor in the binary.
       * no need to apply the following hack.
       */

      return;
    }

  g_saved_argc = argc;
  g_saved_argv = argv;
  g_saved_envp = envp;
  g_saved_apple = apple;

  g_saved_init_funcs = malloc(g_num_saved_init_funcs *
                              sizeof(*g_saved_init_funcs));
  allow_write(&mod_init_func_start, &mod_init_func_end);
  int i = 0;
  for (fp = &mod_init_func_start; fp < &mod_init_func_end; fp++)
    {
      if (*fp == save_and_replace_init_funcs)
        {
          assert(i == 0);
        }
      else
        {
          g_saved_init_funcs[i - 1] = *fp;
          *fp = noop;
        }
      i++;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: macho_call_saved_init_funcs
 ****************************************************************************/

void
macho_call_saved_init_funcs(void)
{
  unsigned int i;
  for (i = 0; i < g_num_saved_init_funcs; i++)
    {
      g_saved_init_funcs[i](g_saved_argc, g_saved_argv, g_saved_envp,
                            g_saved_apple);
    }
}
