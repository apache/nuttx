/****************************************************************************
 * libnxflat/lib/nxflat_stack.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/types.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/nxflat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_adjuststacksize
 ****************************************************************************/

void nxflat_adjuststacksize(struct nxflat_loadinfo_s *load_info,
			     int argc, int envc, int system_usage)
{
  uint32 total_usage = system_usage;

  /* For argc, we will store the array (argc elements), the argc
   * value itself, plus a null pointer.
   */

  total_usage += (argc + 2) * sizeof(uint32);

  /* For envc, we will store the array (envc elements) plus a null
   * pointer.
   */

  total_usage += (envc + 1) * sizeof(uint32);

  /* And we will store six additional words described the memory
   * layout.
   */

  total_usage += 6 * sizeof(uint32);

  /* Add this to the previously determined stack size */

  load_info->stack_size += total_usage;
}

/****************************************************************************
 * Name: nxflat_initstack
 *
 * Description:
 *   When we enter the NXFLAT_ loader, it will expect to see a stack frame
 *   like the following.  NOTE: This logic assumes a push down stack
 *   (i.e., we decrement the stack pointer to go from the "BOTTOM" to
 *   the "TOP").
 *
 *
 *    TOP->argc                Argument count    (integer)
 *         argv[0...(argc-1)]  Program arguments (pointers)
 *         NULL                Marks end of arguments
 *         env[0...N]          Environment variables (pointers)
 *         NULL                Marks end of environment variables
 *         loader ispace       Address of loader ISpace (NXFLAT_ header)
 *         loader dspace       Address of loader DSpace
 *         loader dspace size  Size of the allocated loader DSpace
 *         program ispace      Address of program ISpace (NXFLAT_ header)
 *         program dspace      Address of program DSpace
 * BOTTOM->program dspace size Size of the allocated program DSpace
 *
 ****************************************************************************/

uint32 nxflat_initstack(struct nxflat_loadinfo_s *prog_load_info,
		       struct nxflat_loadinfo_s *lib_load_info,
		       int argc, int envc, char *p)
{
  uint32 *argv;
  uint32 *envp;
  uint32 *sp;
  char       dummy;

  /* p points to the beginning of the array of arguments;
   * sp points to the "bottom" of a push down stack.
   */

  sp = (uint32*)((-(uint32)sizeof(char*))&(uint32) p);

  /* Place program information on the stack */

  if (prog_load_info)
    {
      *sp-- = (uint32)prog_load_info->dspace_size;
      *sp-- = (uint32)prog_load_info->dspace;
      *sp-- = (uint32)prog_load_info->ispace;
    }
  else
    {
      dbg("No program load info provided\n");
      return -EINVAL;
    }

  /* Place loader information on the stack */

  if (lib_load_info)
    {
      *sp-- = (uint32)lib_load_info->dspace_size;
      *sp-- = (uint32)lib_load_info->dspace;
      *sp-- = (uint32)lib_load_info->ispace;
    }
  else
    {
      *sp-- = (uint32)0;
      *sp-- = (uint32)0;
      *sp-- = (uint32)0;
    }

  /* Allocate space on the stack for the envp array contents
   * (including space for a null terminator).
   */

  sp  -= envc+1;
  envp = sp;

  /* Allocate space on the stack for the argv array contents
   * (including space for a null terminator).
   */

  sp  -= argc+1;
  argv = sp;

  /* Put argc on the stack. sp now points to the "top" of the
   * stack as it will be received by the new task.
   */

  *sp-- = (uint32)argc;

  /* Copy argv pointers into the stack frame (terminated with
   * a null pointer).
   */

  prog_load_info->arg_start = (uint32)p;
  while (argc-->0)
    {
      /* Put the address of the beginning of the string */

      *argv++ = (uint32)p;

      /* Search for the end of the string */

      do
	{
	  dummy = *p++;
	}
      while (dummy);
    }
  *argv = (uint32)NULL,argv;

  /* Copy envp pointers into the stack frame (terminated with
   * a null pointer).
   */

  prog_load_info->env_start = (uint32)p;
  while (envc-->0)
    {
      /* Put the address of the beginning of the string */

      *envp++ = (uint32)p;

      /* Search for the end of the string */

      do
	{
	  dummy = *p++;
	}
      while (dummy);
    }
  *envp = (uint32)NULL;

  prog_load_info->env_end = (uint32)p;

  return (uint32)sp;
}

