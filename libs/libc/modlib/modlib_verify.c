/****************************************************************************
 * libs/libc/modlib/modlib_verify.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <elf32.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

static const char g_modmagic[EI_MAGIC_SIZE] =
{
    0x7f, 'E', 'L', 'F'
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_verifyheader
 *
 * Description:
 *   Given the header from a possible ELF executable, verify that it
 *   is an ELF executable.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 *   -ENOEXEC  : Not an ELF file
 *   -EINVAL : Not a relocatable ELF file or not supported by the current,
 *               configured architecture.
 *
 ****************************************************************************/

int modlib_verifyheader(FAR const Elf32_Ehdr *ehdr)
{
  if (!ehdr)
    {
      berr("ERROR: NULL ELF header!");
      return -ENOEXEC;
    }

  /* Verify that the magic number indicates an ELF file */

  if (memcmp(ehdr->e_ident, g_modmagic, EI_MAGIC_SIZE) != 0)
    {
      binfo("Not ELF magic {%02x, %02x, %02x, %02x}\n",
            ehdr->e_ident[0], ehdr->e_ident[1], ehdr->e_ident[2], ehdr->e_ident[3]);
      return -ENOEXEC;
    }

  /* Verify that this is a relocatable file */

  if (ehdr->e_type != ET_REL)
    {
      berr("ERROR: Not a relocatable file: e_type=%d\n", ehdr->e_type);
      return -EINVAL;
    }

  /* Verify that this file works with the currently configured architecture */

  if (!up_checkarch(ehdr))
    {
      berr("ERROR: Not a supported architecture\n");
      return -ENOEXEC;
    }

  /* Looks good so far... we still might find some problems later. */

  return OK;
}
