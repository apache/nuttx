/****************************************************************************
 * examples/nsh/dbg_proccmds.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "nsh.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dbgmem_s
{
  boolean      dm_write;  /* TRUE: perfrom write operation */
  void        *dm_addr;   /* Address to access */
  uint32       dm_value;  /* Value to write */
  unsigned int dm_count;  /* The number of bytes to access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mem_parse
 ****************************************************************************/

int mem_parse(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv,
              struct dbgmem_s *mem)
{
  char *pcvalue = strchr(argv[1], '=');
  unsigned long lvalue = 0;

  /* Check if we are writing a value */

  if (pcvalue)
    {
      *pcvalue = '\0';
      pcvalue++;

      lvalue = (unsigned long)strtol(pcvalue, NULL, 16);
      if (lvalue > 0xffffffff)
        {
          return -EINVAL;
        }

      mem->dm_write = TRUE;
      mem->dm_value = (uint32)lvalue;
    }
  else
    {
      mem->dm_write = FALSE;
      mem->dm_value = 0;
    }

  /* Get the address to be accessed */

  mem->dm_addr = (void*)strtol(argv[1], NULL, 16);

  /* Get the number of bytes to access */

  if (argc > 2)
    {
      mem->dm_count = (unsigned int)strtol(argv[2], NULL, 16);
    }
  else
    {
      mem->dm_count = 1;
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_mb
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NSH_DISABLE_MB
int cmd_mb(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct dbgmem_s mem;
  volatile ubyte *ptr;
  int ret;
  int i;

  ret = mem_parse(vtbl, argc, argv, &mem);
  if (ret == 0)
    {
      /* Loop for the number of requested bytes */

      for (i = 0, ptr = (volatile ubyte*)mem.dm_addr; i < mem.dm_count; i++, ptr++)
	{
	  /* Print the value at the address */

	  nsh_output(vtbl, "  %p = 0x%02x", ptr, *ptr);

	  /* Are we supposed to write a value to this address? */

	  if (mem.dm_write)
	    {
	      /* Yes, was the supplied value within range? */

	      if (mem.dm_value > 0x000000ff)
		{
		  nsh_output(vtbl, g_fmtargrange, argv[0]);
		  return ERROR;
		}

	      /* Write the value and re-read the address so that we print its
	       * current value (if the address is a process address, then the
	       * value read might not necessarily be the value written).
	       */

	      *ptr = (ubyte)mem.dm_value;
	      nsh_output(vtbl, " -> 0x%02x", *ptr);
	    }

	  /* Make sure we end it with a newline */

	  nsh_output(vtbl, "\n", *ptr);
	}
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: cmd_mh
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NSH_DISABLE_MH
int cmd_mh(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct dbgmem_s mem;
  volatile uint16 *ptr;
  int ret;
  int i;

  ret = mem_parse(vtbl, argc, argv, &mem);
  if (ret == 0)
    {
      /* Loop for the number of requested bytes */

      for (i = 0, ptr = (volatile uint16*)mem.dm_addr; i < mem.dm_count; i += 2, ptr++)
	{
	  /* Print the value at the address */

	  nsh_output(vtbl, "  %p = 0x%04x", ptr, *ptr);

	  /* Are we supposed to write a value to this address? */

	  if (mem.dm_write)
	    {
	      /* Yes, was the supplied value within range? */

	      if (mem.dm_value > 0x0000ffff)
		{
		  nsh_output(vtbl, g_fmtargrange, argv[0]);
		  return ERROR;
		}

	      /* Write the value and re-read the address so that we print its
	       * current value (if the address is a process address, then the
	       * value read might not necessarily be the value written).
	       */

	      *ptr = (uint16)mem.dm_value;
	      nsh_output(vtbl, " -> 0x%04x", *ptr);
	    }

	  /* Make sure we end it with a newline */

	  nsh_output(vtbl, "\n", *ptr);
	}
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: cmd_mw
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NSH_DISABLE_MW
int cmd_mw(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct dbgmem_s mem;
  volatile uint32 *ptr;
  int ret;
  int i;

  ret = mem_parse(vtbl, argc, argv, &mem);
  if (ret == 0)
    {
      /* Loop for the number of requested bytes */

      for (i = 0, ptr = (volatile uint32*)mem.dm_addr; i < mem.dm_count; i += 4, ptr++)
	{
	  /* Print the value at the address */

	  nsh_output(vtbl, "  %p = 0x%08x", ptr, *ptr);

	  /* Are we supposed to write a value to this address? */

	  if (mem.dm_write)
	    {
	      /* Write the value and re-read the address so that we print its
	       * current value (if the address is a process address, then the
	       * value read might not necessarily be the value written).
	       */

	      *ptr = mem.dm_value;
	      nsh_output(vtbl, " -> 0x%08x", *ptr);
	    }

	  /* Make sure we end it with a newline */

	  nsh_output(vtbl, "\n", *ptr);
	}
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: cmd_mem
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NSH_DISABLE_MEM
int cmd_mem(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct mallinfo mem;

#ifdef CONFIG_CAN_PASS_STRUCTS
  mem = mallinfo();
#else
  (void)mallinfo(&mem);
#endif

  nsh_output(vtbl, "  arena:    %8x\n", mem.arena);
  nsh_output(vtbl, "  ordblks:  %8d\n", mem.ordblks);
  nsh_output(vtbl, "  mxordblk: %8x\n", mem.mxordblk);
  nsh_output(vtbl, "  uordblks: %8x\n", mem.uordblks);
  nsh_output(vtbl, "  fordblks: %8x\n", mem.fordblks);
  return OK;
}
#endif

/****************************************************************************
 * Name: nsh_dumpbuffer
 ****************************************************************************/

void nsh_dumpbuffer(FAR struct nsh_vtbl_s *vtbl, const char *msg,
                    const ubyte *buffer, ssize_t nbytes)
{
  char line[128];
  int ch;
  int i;
  int j;

  nsh_output(vtbl, "%s:\n", msg);
  for (i = 0; i < nbytes; i += 16)
    {
      sprintf(line, "%04x: ", i);

      for ( j = 0; j < 16; j++)
        {
          if (i + j < nbytes)
            {
              sprintf(&line[strlen(line)], "%02x ", buffer[i+j] );
            }
          else
            {
              strcpy(&line[strlen(line)], "   ");
            }
        }

      for ( j = 0; j < 16; j++)
        {
          if (i + j < nbytes)
            {
              ch = buffer[i+j];
              sprintf(&line[strlen(line)], "%c", ch >= 0x20 && ch <= 0x7e ? ch : '.');
            }
        }
      nsh_output(vtbl, "%s\n", line);
    }
}

/****************************************************************************
 * Name: cmd_xd
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_NSH_DISABLE_XD
int cmd_xd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char  *addr;
  char  *endptr;
  int    nbytes;

  addr = (char*)strtol(argv[1], &endptr, 16);
  if (argv[0][0] == '\0' || *endptr != '\0')
    {
      return ERROR;
    }

  nbytes = (int)strtol(argv[2], &endptr, 0);
  if (argv[0][0] == '\0' || *endptr != '\0' || nbytes < 0)
    {
      return ERROR;
    }

  nsh_dumpbuffer(vtbl, "Hex dump", (ubyte*)addr, nbytes);
  return OK;
}
#endif

