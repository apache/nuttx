/****************************************************************************
 * libs/libc/modlib/modlib_verify.c
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

#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/elf.h>
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

int modlib_verifyheader(FAR const Elf_Ehdr *ehdr)
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
            ehdr->e_ident[0], ehdr->e_ident[1], ehdr->e_ident[2],
            ehdr->e_ident[3]);
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
