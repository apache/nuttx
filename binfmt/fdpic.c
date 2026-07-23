/****************************************************************************
 * binfmt/fdpic.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * Loader for FDPIC ELF modules, for executing code in place out of flash.
 *
 * An FDPIC module has two loadable segments that may be placed completely
 * independently of one another.  That is the property this loader exists to
 * exploit: the read-only segment is *mapped* rather than copied, so on a
 * filesystem that can hand out a direct pointer to the media -- xipfs, or
 * ROMFS -- a module's text and rodata are executed and read where they
 * already sit in flash, and never occupy RAM.  Only the writable segment is
 * copied, once per running instance.
 *
 * Because the two segments are unrelated at run time, a function's address
 * is not enough to call it: the callee also needs its data base.  FDPIC
 * therefore represents a function pointer as a *descriptor*, the pair
 * {entry, GOT}, and the caller installs the GOT half into the FDPIC
 * register before branching.  Building those descriptors is most of what
 * this loader does.
 *
 * The relocation set is small because the static link has already folded
 * the GOT-relative work away.  Only three kinds survive into the module:
 *
 *   R_ARM_RELATIVE       an address that needs its segment's base added
 *   R_ARM_FUNCDESC_VALUE a descriptor to fill in, usually for an import
 *   R_ARM_FUNCDESC       a pointer to a descriptor the loader must supply
 *
 * A module's '.rofixup' section is deliberately ignored: it exists for a
 * static executable's crt0, which a module never runs, and holds only the
 * GOT self-pointer that this loader supplies from the register instead.
 *
 * The base firmware must be built with the FDPIC register reserved
 * (-ffixed-r9 on ARM); a well behaved module is not enough, because a kernel
 * routine calling back into module code has to arrive with the module's GOT
 * still in the register.
 *
 * Documentation/components/fdpic.rst has the reasoning behind all of this in
 * full, including why ignoring '.rofixup' is correct rather than tolerated.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mman.h>
#include <sys/types.h>

#include <debug.h>
#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <arch/elf.h>

#include <nuttx/arch.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/binfmt/fdpic.h>
#include <nuttx/elf.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/symtab.h>

#ifdef CONFIG_FDPIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ELFOSABI_ARM_FDPIC
#  define ELFOSABI_ARM_FDPIC  65
#endif

/* The Thumb bit lives in the low bit of a code address.  Descriptors and
 * entry points must keep it or the core faults on the branch.
 */

#define FDPIC_THUMB_BIT       1

/* Calling into a module from the loader -- which is what running its
 * constructors amounts to -- needs the object's data base installed in the
 * FDPIC register first, and that takes a little assembly.  Where it cannot
 * be done, a module carrying constructors is refused rather than run with
 * whatever the register happened to hold.
 */

#if defined(CONFIG_ARCH_ARM) && defined(__thumb__)
#  define FDPIC_HAVE_CALLFN   1
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int fdpic_loadbinary(FAR struct binary_s *binp,
                            FAR const char *filename,
                            FAR const struct symtab_s *exports,
                            int nexports);
static int fdpic_unloadbinary(FAR struct binary_s *binp);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct binfmt_s g_fdpicbinfmt =
{
  NULL,               /* next */
  fdpic_loadbinary,   /* load */
  fdpic_unloadbinary, /* unload */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdpic_read
 *
 * Description:
 *   Read from the module file at an absolute offset.
 *
 ****************************************************************************/

static int fdpic_read(FAR struct fdpic_loadinfo_s *loadinfo,
                      FAR void *buffer, size_t nbytes, off_t offset)
{
  FAR uint8_t *dest = buffer;
  ssize_t nread;

  if (file_seek(&loadinfo->file, offset, SEEK_SET) < 0)
    {
      return -EIO;
    }

  while (nbytes > 0)
    {
      nread = file_read(&loadinfo->file, dest, nbytes);
      if (nread < 0)
        {
          if (nread == -EINTR)
            {
              continue;
            }

          return (int)nread;
        }

      if (nread == 0)
        {
          return -ENODATA;
        }

      dest   += nread;
      nbytes -= nread;
    }

  return OK;
}

/****************************************************************************
 * Name: fdpic_addr
 *
 * Description:
 *   Translate a link-time virtual address into the address the segment
 *   actually landed at.  This is the whole of the FDPIC "load map": two
 *   segments, each with its own independent bias.
 *
 * Returned Value:
 *   The run-time address, or 0 if the address belongs to neither segment.
 *
 ****************************************************************************/

static uintptr_t fdpic_addr(FAR struct fdpic_loadinfo_s *loadinfo,
                            uintptr_t vaddr)
{
  if (vaddr >= loadinfo->textvaddr &&
      vaddr <  loadinfo->textvaddr + loadinfo->textsize)
    {
      return loadinfo->textaddr + (vaddr - loadinfo->textvaddr);
    }

  if (vaddr >= loadinfo->datavaddr &&
      vaddr <  loadinfo->datavaddr + loadinfo->datamemsz)
    {
      return loadinfo->dataaddr + (vaddr - loadinfo->datavaddr);
    }

  return 0;
}

/****************************************************************************
 * Name: fdpic_verify
 *
 * Description:
 *   Check that this really is an FDPIC module for this machine.
 *
 *   The FDPIC marker is the OS/ABI byte, not e_flags: a module's e_flags
 *   are an unremarkable EABI version, so testing them would reject every
 *   valid module.
 *
 ****************************************************************************/

static int fdpic_verify(FAR const Elf32_Ehdr *ehdr)
{
  if (memcmp(ehdr->e_ident, ELFMAG, EI_MAGIC_SIZE) != 0)
    {
      return -ENOEXEC;
    }

  if (ehdr->e_ident[EI_CLASS] != ELFCLASS32 ||
      ehdr->e_ident[EI_DATA] != ELFDATA2LSB)
    {
      berr("ERROR: Not a little-endian 32-bit object\n");
      return -ENOEXEC;
    }

  if (ehdr->e_ident[EI_OSABI] != ELFOSABI_ARM_FDPIC)
    {
      /* Not FDPIC.  Quietly decline so another binfmt handler can try. */

      return -ENOEXEC;
    }

  if (ehdr->e_type != ET_DYN)
    {
      berr("ERROR: FDPIC module must be ET_DYN, got %u\n", ehdr->e_type);
      return -ENOEXEC;
    }

  if (ehdr->e_machine != EM_ARM)
    {
      berr("ERROR: Not an ARM module (e_machine=%u)\n", ehdr->e_machine);
      return -ENOEXEC;
    }

  return OK;
}

/****************************************************************************
 * Name: fdpic_readdynamic
 *
 * Description:
 *   Pull the handful of dynamic tags the loader needs out of PT_DYNAMIC.
 *
 *   This is read straight from the file rather than from a loaded segment,
 *   because it has to be known before the writable segment can be sized:
 *   the relocation count bounds how many descriptors might be requested.
 *
 ****************************************************************************/

static int fdpic_readdynamic(FAR struct fdpic_loadinfo_s *loadinfo,
                             FAR const Elf32_Phdr *phdr)
{
  FAR Elf32_Dyn *dyn;
  uint32_t pltreltype = DT_REL;
  size_t ndyn;
  size_t i;
  int ret;

  loadinfo->nneeded = 0;

  if (phdr->p_filesz == 0 || phdr->p_filesz > 4096)
    {
      return -ENOEXEC;
    }

  dyn = kmm_malloc(phdr->p_filesz);
  if (dyn == NULL)
    {
      return -ENOMEM;
    }

  ret = fdpic_read(loadinfo, dyn, phdr->p_filesz, phdr->p_offset);
  if (ret < 0)
    {
      kmm_free(dyn);
      return ret;
    }

  ndyn = phdr->p_filesz / sizeof(Elf32_Dyn);

  for (i = 0; i < ndyn && dyn[i].d_tag != DT_NULL; i++)
    {
      switch (dyn[i].d_tag)
        {
          case DT_REL:
            loadinfo->relvaddr = dyn[i].d_un.d_ptr;
            break;

          case DT_RELSZ:
            loadinfo->relsize = dyn[i].d_un.d_val;
            break;

          case DT_SYMTAB:
            loadinfo->symtabvaddr = dyn[i].d_un.d_ptr;
            break;

          case DT_STRTAB:
            loadinfo->strtabvaddr = dyn[i].d_un.d_ptr;
            break;

          case DT_PLTGOT:
            loadinfo->gotaddr = dyn[i].d_un.d_ptr;
            break;

          case DT_JMPREL:
            loadinfo->pltrelvaddr = dyn[i].d_un.d_ptr;
            break;

          case DT_PLTRELSZ:
            loadinfo->pltrelsize = dyn[i].d_un.d_val;
            break;

          case DT_PLTREL:
            pltreltype = dyn[i].d_un.d_val;
            break;

          case DT_INIT_ARRAY:
            loadinfo->initvaddr = dyn[i].d_un.d_ptr;
            break;

          case DT_INIT_ARRAYSZ:
            loadinfo->initsize = dyn[i].d_un.d_val;
            break;

          case DT_FINI_ARRAY:
            loadinfo->finivaddr = dyn[i].d_un.d_ptr;
            break;

          case DT_FINI_ARRAYSZ:
            loadinfo->finisize = dyn[i].d_un.d_val;
            break;

          case DT_NEEDED:
            /* An offset into DT_STRTAB.  The string itself lives in the
             * read-only segment, which is not mapped yet, so only the
             * offset is recorded here.
             */

            if (loadinfo->nneeded < FDPIC_MAX_NEEDED)
              {
                loadinfo->needed[loadinfo->nneeded++] = dyn[i].d_un.d_val;
              }
            else
              {
                berr("ERROR: More than %d DT_NEEDED entries\n",
                     FDPIC_MAX_NEEDED);
                kmm_free(dyn);
                return -ENOEXEC;
              }
            break;

          default:
            break;
        }
    }

  kmm_free(dyn);

  /* DT_PLTREL says which format the PLT table is in.  Nothing anywhere in
   * this loader reads RELA, so an object claiming that layout must be
   * refused rather than walked as if it were REL: the entries are 12 bytes,
   * not 8, and misreading them would apply garbage relocations.
   */

  if (loadinfo->pltrelsize != 0 && pltreltype != DT_REL)
    {
      berr("ERROR: %s has RELA PLT relocations, which are not supported\n",
           loadinfo->name);
      return -ENOEXEC;
    }

  if (loadinfo->gotaddr == 0)
    {
      /* DT_PLTGOT is only emitted when the object has a PLT, which a leaf
       * library calling nothing external does not.  It still has a GOT --
       * its own data is reached through it, so the FDPIC register has to
       * point at it -- and the linker places that GOT immediately after
       * the dynamic section.
       */

      loadinfo->gotaddr = phdr->p_vaddr + phdr->p_memsz;

      binfo("fdpic: %s has no DT_PLTGOT, taking GOT at %08lx\n",
            loadinfo->name, (unsigned long)loadinfo->gotaddr);
    }

  return OK;
}

/****************************************************************************
 * Name: fdpic_load
 *
 * Description:
 *   Place both segments: map the read-only one, copy the writable one.
 *
 ****************************************************************************/

static int fdpic_load(FAR struct fdpic_loadinfo_s *loadinfo)
{
  FAR Elf32_Phdr *phdrs;
  FAR Elf32_Phdr *text = NULL;
  FAR Elf32_Phdr *data = NULL;
  FAR Elf32_Phdr *dynamic = NULL;
  uintptr_t mapped = 0;
  size_t phdrsize;
  size_t i;
  int ret;

  if (loadinfo->ehdr.e_phnum == 0 ||
      loadinfo->ehdr.e_phentsize != sizeof(Elf32_Phdr))
    {
      return -ENOEXEC;
    }

  phdrsize = (size_t)loadinfo->ehdr.e_phnum * sizeof(Elf32_Phdr);
  phdrs    = kmm_malloc(phdrsize);
  if (phdrs == NULL)
    {
      return -ENOMEM;
    }

  ret = fdpic_read(loadinfo, phdrs, phdrsize, loadinfo->ehdr.e_phoff);
  if (ret < 0)
    {
      goto errout_with_phdrs;
    }

  for (i = 0; i < loadinfo->ehdr.e_phnum; i++)
    {
      if (phdrs[i].p_type == PT_DYNAMIC)
        {
          dynamic = &phdrs[i];
        }
      else if (phdrs[i].p_type == PT_LOAD)
        {
          if ((phdrs[i].p_flags & PF_W) != 0)
            {
              data = &phdrs[i];
            }
          else
            {
              text = &phdrs[i];
            }
        }
    }

  if (text == NULL || data == NULL || dynamic == NULL)
    {
      berr("ERROR: Expected one RX and one RW PT_LOAD plus PT_DYNAMIC\n");
      ret = -ENOEXEC;
      goto errout_with_phdrs;
    }

  loadinfo->textvaddr  = text->p_vaddr;
  loadinfo->textsize   = text->p_memsz;
  loadinfo->datavaddr  = data->p_vaddr;
  loadinfo->datafilesz = data->p_filesz;
  loadinfo->datamemsz  = data->p_memsz;

  ret = fdpic_readdynamic(loadinfo, dynamic);
  if (ret < 0)
    {
      goto errout_with_phdrs;
    }

  /* Map the read-only segment.
   *
   * MAP_XIP_STRICT is the point of the exercise: it tells the filesystem
   * to resolve the mapping onto the media or fail, rather than quietly
   * copying the file into RAM.  A silent copy would still work, which is
   * exactly why it must be refused -- it would cost the RAM this loader
   * exists to save, and nothing would report it.
   */

  ret = file_ioctl(&loadinfo->file, XIPFSIOC_PIN,
                   (unsigned long)(uintptr_t)&mapped);
  if (ret >= 0)
    {
      /* The pin is held until unload, which happens when the spawned task
       * exits.  That is a different task from the one running the loader,
       * so this deliberately does not go through mmap: an mm_map entry
       * would be recorded against whoever called the loader and would
       * never be released by the module's own exit.
       */

      loadinfo->textaddr   = (uintptr_t)mapped + text->p_offset;
      loadinfo->textmapped = true;
    }
#ifdef CONFIG_FDPIC_ALLOW_COPY
  else
    {
      /* Bring-up escape hatch: keep working on a filesystem that cannot
       * expose its media, at the cost of the RAM saving.
       */

      fwarn("fdpic: cannot execute in place (%d), copying text to RAM\n",
            ret);

      loadinfo->textaddr = (uintptr_t)kmm_malloc(text->p_memsz);
      if (loadinfo->textaddr == 0)
        {
          ret = -ENOMEM;
          goto errout_with_phdrs;
        }

      ret = fdpic_read(loadinfo, (FAR void *)loadinfo->textaddr,
                       text->p_filesz, text->p_offset);
      if (ret < 0)
        {
          goto errout_with_text;
        }
    }
#else
  else
    {
      berr("ERROR: Module cannot be executed in place: %d\n", ret);
      goto errout_with_phdrs;
    }
#endif

  /* Allocate the writable segment, with room after it for any descriptors
   * the relocations ask us to manufacture.  Bounding that by the total
   * relocation count costs a few bytes and avoids a second pass over the
   * relocation table.
   *
   * Both tables have to be counted.  An R_ARM_FUNCDESC in the PLT table
   * draws from the same pool, so sizing this from DT_RELSZ alone would let
   * a module carrying DT_JMPREL entries run off the end of the allocation
   * and corrupt the heap.
   */

  loadinfo->ndesc     = (loadinfo->relsize + loadinfo->pltrelsize) /
                        sizeof(Elf32_Rel);
  loadinfo->dataalloc = loadinfo->datamemsz +
                        (size_t)loadinfo->ndesc *
                        sizeof(struct fdpic_desc_s);

  loadinfo->dataaddr = (uintptr_t)kumm_zalloc(loadinfo->dataalloc);
  if (loadinfo->dataaddr == 0)
    {
      ret = -ENOMEM;
      goto errout_with_text;
    }

  loadinfo->descpool = loadinfo->dataaddr + loadinfo->datamemsz;

  ret = fdpic_read(loadinfo, (FAR void *)loadinfo->dataaddr,
                   loadinfo->datafilesz, data->p_offset);
  if (ret < 0)
    {
      goto errout_with_data;
    }

  /* Everything past p_filesz is .bss, already zeroed by kumm_zalloc */

  loadinfo->gotaddr = fdpic_addr(loadinfo, loadinfo->gotaddr);
  loadinfo->entry   = fdpic_addr(loadinfo,
                                 loadinfo->ehdr.e_entry & ~FDPIC_THUMB_BIT);

  if (loadinfo->gotaddr == 0 || loadinfo->entry == 0)
    {
      berr("ERROR: GOT or entry point outside any segment\n");
      ret = -ENOEXEC;
      goto errout_with_data;
    }

  loadinfo->entry |= (loadinfo->ehdr.e_entry & FDPIC_THUMB_BIT);

  binfo("fdpic: text %08lx (%s) data %08lx got %08lx entry %08lx\n",
        (unsigned long)loadinfo->textaddr,
        loadinfo->textmapped ? "in place" : "copied",
        (unsigned long)loadinfo->dataaddr,
        (unsigned long)loadinfo->gotaddr,
        (unsigned long)loadinfo->entry);

  kmm_free(phdrs);
  return OK;

errout_with_data:
  kumm_free((FAR void *)loadinfo->dataaddr);
  loadinfo->dataaddr = 0;

errout_with_text:
  if (loadinfo->textmapped)
    {
      file_ioctl(&loadinfo->file, XIPFSIOC_UNPIN, 0);
    }
  else if (loadinfo->textaddr != 0)
    {
      kmm_free((FAR void *)loadinfo->textaddr);
    }

  loadinfo->textaddr = 0;

errout_with_phdrs:
  kmm_free(phdrs);
  return ret;
}

/****************************************************************************
 * Name: fdpic_release
 *
 * Description:
 *   Release every object of a load.  Dropping a text pin is what lets a
 *   compacting filesystem move those blocks again, now that nothing is
 *   executing from them.
 *
 ****************************************************************************/

static void fdpic_release(FAR struct fdpic_loadinfo_s *head)
{
  FAR struct fdpic_loadinfo_s *obj;
  FAR struct fdpic_loadinfo_s *next;

  for (obj = head; obj != NULL; obj = next)
    {
      next = obj->flink;

      if (obj->dataaddr != 0)
        {
          kumm_free((FAR void *)obj->dataaddr);
        }

      if (obj->textmapped)
        {
          file_ioctl(&obj->file, XIPFSIOC_UNPIN, 0);
        }
      else if (obj->textaddr != 0)
        {
          kmm_free((FAR void *)obj->textaddr);
        }

      file_close(&obj->file);
      kmm_free(obj);
    }
}

/****************************************************************************
 * Name: fdpic_loadobject
 *
 * Description:
 *   Open one object, place its segments, and append it to the load's list.
 *
 ****************************************************************************/

static int fdpic_loadobject(FAR const char *path, FAR const char *name,
                            FAR struct fdpic_loadinfo_s **head,
                            FAR struct fdpic_loadinfo_s **out)
{
  FAR struct fdpic_loadinfo_s *obj;
  FAR struct fdpic_loadinfo_s *tail;
  int ret;

  obj = kmm_zalloc(sizeof(struct fdpic_loadinfo_s));
  if (obj == NULL)
    {
      return -ENOMEM;
    }

  strlcpy(obj->name, name, sizeof(obj->name));

  ret = file_open(&obj->file, path, O_RDONLY);
  if (ret < 0)
    {
      /* Not an error worth shouting about: binfmt offers every candidate
       * name to every loader, including builtin command names that are
       * not files at all.
       */

      binfo("fdpic: cannot open %s: %d\n", path, ret);
      kmm_free(obj);
      return ret;
    }

  ret = fdpic_read(obj, &obj->ehdr, sizeof(Elf32_Ehdr), 0);
  if (ret >= 0)
    {
      ret = fdpic_verify(&obj->ehdr);
    }

  if (ret >= 0)
    {
      ret = fdpic_load(obj);
    }

  if (ret < 0)
    {
      file_close(&obj->file);
      kmm_free(obj);
      return ret;
    }

  /* Append, so the module stays first and dependencies follow it */

  if (*head == NULL)
    {
      *head = obj;
    }
  else
    {
      for (tail = *head; tail->flink != NULL; tail = tail->flink)
        {
        }

      tail->flink = obj;
    }

  if (out != NULL)
    {
      *out = obj;
    }

  return OK;
}

/****************************************************************************
 * Name: fdpic_loaddepends
 *
 * Description:
 *   Load whatever this object names in DT_NEEDED.
 *
 *   The names live in the string table inside the read-only segment, so
 *   this can only run once that segment is mapped.  Anything already
 *   loaded for this module is reused rather than loaded twice.
 *
 *   That reuse is also what makes a dependency cycle harmless: an object is
 *   on the list before its own dependencies are walked, so a library naming
 *   something already loaded finds it and stops.  What the list cannot
 *   bound is a chain of distinct names, which is why the depth is capped
 *   explicitly.
 *
 ****************************************************************************/

static int fdpic_loaddepends(FAR struct fdpic_loadinfo_s *obj,
                             FAR struct fdpic_loadinfo_s **head,
                             int depth)
{
  FAR const char *strtab;
  FAR struct fdpic_loadinfo_s *o;
  FAR struct fdpic_loadinfo_s *dep;
  char path[128];
  bool seen;
  int ret;
  int i;

  if (obj->nneeded == 0)
    {
      return OK;
    }

  if (depth >= FDPIC_MAX_DEPTH)
    {
      berr("ERROR: %s exceeds the %d level DT_NEEDED depth limit\n",
           obj->name, FDPIC_MAX_DEPTH);
      return -ELOOP;
    }

  strtab = (FAR const char *)fdpic_addr(obj, obj->strtabvaddr);
  if (strtab == NULL)
    {
      return -ENOEXEC;
    }

  for (i = 0; i < obj->nneeded; i++)
    {
      FAR const char *name = &strtab[obj->needed[i]];

      seen = false;
      for (o = *head; o != NULL; o = o->flink)
        {
          if (strcmp(o->name, name) == 0)
            {
              seen = true;
              break;
            }
        }

      if (seen)
        {
          continue;
        }

      snprintf(path, sizeof(path), "%s/%s", CONFIG_FDPIC_LIBPATH, name);

      binfo("fdpic: %s needs %s\n", obj->name, name);

      ret = fdpic_loadobject(path, name, head, &dep);
      if (ret < 0)
        {
          berr("ERROR: Cannot load %s needed by %s: %d\n",
               name, obj->name, ret);
          return ret;
        }

      /* Libraries may depend on libraries */

      ret = fdpic_loaddepends(dep, head, depth + 1);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: fdpic_symvalue
 *
 * Description:
 *   Resolve one relocation's symbol.  A symbol defined inside the module is
 *   translated through the load map; an undefined one is an import and is
 *   looked up in the symbol table the base firmware exports.
 *
 ****************************************************************************/

static int fdpic_symvalue(FAR struct fdpic_loadinfo_s *obj,
                          FAR struct fdpic_loadinfo_s *head,
                          uint32_t symidx,
                          FAR const struct symtab_s *exports, int nexports,
                          FAR uintptr_t *value,
                          FAR struct fdpic_loadinfo_s **owner)
{
  FAR const Elf32_Sym *symtab;
  FAR const Elf32_Sym *sym;
  FAR const char *strtab;
  FAR const char *name;
  FAR const struct symtab_s *found;
  FAR struct fdpic_loadinfo_s *o;

  symtab = (FAR const Elf32_Sym *)fdpic_addr(obj, obj->symtabvaddr);
  strtab = (FAR const char *)fdpic_addr(obj, obj->strtabvaddr);

  if (symtab == NULL || strtab == NULL)
    {
      return -ENOEXEC;
    }

  sym  = &symtab[symidx];
  name = &strtab[sym->st_name];

  if (sym->st_shndx != SHN_UNDEF)
    {
      /* Defined right here */

      *value = fdpic_addr(obj, sym->st_value & ~FDPIC_THUMB_BIT);
      if (*value == 0)
        {
          berr("ERROR: '%s' outside any segment of %s\n", name, obj->name);
          return -ENOEXEC;
        }

      *value |= (sym->st_value & FDPIC_THUMB_BIT);
      *owner  = obj;
      return OK;
    }

  /* Undefined here.  Try the other objects in this load before falling
   * back to the firmware, so a module linked against a library binds to
   * the library rather than to a same-named firmware symbol.
   */

  for (o = head; o != NULL; o = o->flink)
    {
      FAR const Elf32_Sym *osym;
      FAR const Elf32_Sym *otab;
      FAR const char *ostr;
      uint32_t n;
      uint32_t i;

      if (o == obj || o->symtabvaddr == 0)
        {
          continue;
        }

      otab = (FAR const Elf32_Sym *)fdpic_addr(o, o->symtabvaddr);
      ostr = (FAR const char *)fdpic_addr(o, o->strtabvaddr);
      if (otab == NULL || ostr == NULL)
        {
          continue;
        }

      /* The dynamic symbol table runs from its start to the string table,
       * which the linker places immediately after it.
       */

      n = (o->strtabvaddr - o->symtabvaddr) / sizeof(Elf32_Sym);

      for (i = 0; i < n; i++)
        {
          osym = &otab[i];

          if (osym->st_shndx == SHN_UNDEF || osym->st_name == 0)
            {
              continue;
            }

          if (strcmp(&ostr[osym->st_name], name) != 0)
            {
              continue;
            }

          *value = fdpic_addr(o, osym->st_value & ~FDPIC_THUMB_BIT);
          if (*value == 0)
            {
              continue;
            }

          *value |= (osym->st_value & FDPIC_THUMB_BIT);
          *owner  = o;

          binfo("fdpic: '%s' resolved in %s\n", name, o->name);
          return OK;
        }
    }

  /* Imported from the base firmware */

  if (exports == NULL)
    {
      berr("ERROR: '%s' imported but no symbol table was provided\n", name);
      return -ENOENT;
    }

  found = symtab_findbyname(exports, name, nexports);
  if (found == NULL)
    {
      berr("ERROR: Imported symbol '%s' not found\n", name);
      return -ENOENT;
    }

  *value = (uintptr_t)found->sym_value;
  *owner = NULL;               /* Firmware: no GOT of its own */
  return OK;
}

/****************************************************************************
 * Name: fdpic_reltable
 *
 * Description:
 *   Apply one REL relocation table.
 *
 * Input Parameters:
 *   pltrel - This is the DT_JMPREL table, so the words being overwritten
 *            are not addends.  See the addend handling below.
 *
 ****************************************************************************/

static int fdpic_reltable(FAR struct fdpic_loadinfo_s *loadinfo,
                          FAR struct fdpic_loadinfo_s *head,
                          FAR const struct symtab_s *exports, int nexports,
                          uintptr_t relvaddr, size_t relsize,
                          bool pltrel, FAR const char *what)
{
  FAR const Elf32_Rel *rels;
  size_t nrels;
  size_t i;
  int ret;

  if (relsize == 0)
    {
      return OK;
    }

  rels = (FAR const Elf32_Rel *)fdpic_addr(loadinfo, relvaddr);
  if (rels == NULL)
    {
      berr("ERROR: %s of %s is outside any segment\n", what, loadinfo->name);
      return -ENOEXEC;
    }

  nrels = relsize / sizeof(Elf32_Rel);

  for (i = 0; i < nrels; i++)
    {
      uint32_t type   = ELF32_R_TYPE(rels[i].r_info);
      uint32_t symidx = ELF32_R_SYM(rels[i].r_info);
      FAR struct fdpic_loadinfo_s *owner = NULL;
      FAR uint32_t *where;
      uintptr_t value;

      where = (FAR uint32_t *)fdpic_addr(loadinfo, rels[i].r_offset);
      if (where == NULL)
        {
          berr("ERROR: Relocation %zu targets %08lx, outside any segment\n",
               i, (unsigned long)rels[i].r_offset);
          return -ENOEXEC;
        }

      /* Everything relocated must land in the writable segment.  A module
       * that asked us to patch its text could not be executed in place by
       * a second instance, so refuse rather than silently give up sharing.
       */

      if ((uintptr_t)where < loadinfo->dataaddr ||
          (uintptr_t)where >= loadinfo->dataaddr + loadinfo->dataalloc)
        {
          berr("ERROR: Relocation %zu would write outside the RW segment\n",
               i);
          return -ENOEXEC;
        }

      switch (type)
        {
          case R_ARM_NONE:
            break;

          case R_ARM_RELATIVE:
            {
              /* REL format: the addend is the link-time address already
               * sitting at the target.
               */

              uintptr_t addr = fdpic_addr(loadinfo, *where);

              if (addr == 0)
                {
                  berr("ERROR: RELATIVE target %08lx outside any segment\n",
                       (unsigned long)*where);
                  return -ENOEXEC;
                }

              *where = (uint32_t)addr;
            }
            break;

          case R_ARM_FUNCDESC_VALUE:
            {
              /* The target *is* the descriptor: two words, entry then the
               * data base to install before branching.
               *
               * The GOT written here is this module's own, including for
               * imported functions.  That is deliberate and is what makes
               * a callback work: when the base firmware's qsort() calls
               * back into the module's comparison function, the module
               * needs its own data base in the FDPIC register.
               */

              FAR struct fdpic_desc_s *desc =
                (FAR struct fdpic_desc_s *)where;
              uint32_t addend;

              ret = fdpic_symvalue(loadinfo, head, symidx, exports,
                                   nexports, &value, &owner);
              if (ret < 0)
                {
                  return ret;
                }

              /* REL format keeps the addend in place, in the word that is
               * about to become the entry point.  It matters: a static
               * function is referenced through its *section* symbol, whose
               * value is the section base, with the offset -- and the
               * Thumb bit -- carried entirely by the addend.  Dropping it
               * yields an even address and the core faults trying to
               * execute it as ARM code.
               *
               * Except in the DT_JMPREL table, where that word is not an
               * addend at all.  A lazy descriptor is pre-loaded with the
               * address of its PLT resolution stub and a GOT half of -1,
               * for a resolver to overwrite on first call.  Adding that
               * stub address to the symbol value produces an arbitrary
               * address, and the module faults on its first call exactly as
               * if the relocation had never been applied -- which is a
               * remarkably good imitation of the bug this table support was
               * added to fix.  An eager binder overwrites the descriptor
               * outright.
               */

              addend = pltrel ? 0 : desc->entry;

              desc->entry = value + addend;

              /* The GOT half must be the *defining* object's, not this
               * one's.  Calling into a shared library installs that
               * library's data base, which is how it reaches its own
               * globals -- and is the entire reason FDPIC exists.  A
               * firmware symbol has no GOT, so leave this object's in
               * place: the firmware ignores the register, and anything it
               * calls back into belongs to this object.
               */

              desc->got = (owner != NULL) ? owner->gotaddr
                                          : loadinfo->gotaddr;
            }
            break;

          case R_ARM_FUNCDESC:
            {
              /* A pointer to a descriptor, which the loader has to supply.
               * Carve one out of the pool reserved behind the writable
               * segment.
               */

              FAR struct fdpic_desc_s *desc;

              if (loadinfo->usedesc >= loadinfo->ndesc)
                {
                  berr("ERROR: Out of function descriptors\n");
                  return -ENOMEM;
                }

              ret = fdpic_symvalue(loadinfo, head, symidx, exports,
                                   nexports, &value, &owner);
              if (ret < 0)
                {
                  return ret;
                }

              desc = (FAR struct fdpic_desc_s *)loadinfo->descpool +
                     loadinfo->usedesc++;

              desc->entry = value + (pltrel ? 0 : *where);  /* as above */
              desc->got   = (owner != NULL) ? owner->gotaddr
                                            : loadinfo->gotaddr;

              *where = (uint32_t)(uintptr_t)desc;
            }
            break;

          default:
            berr("ERROR: Unsupported relocation type %" PRIu32 "\n", type);
            return -ENOSYS;
        }
    }

  binfo("fdpic: %s: applied %zu %s relocations, %u descriptors created\n",
        loadinfo->name, nrels, what, loadinfo->usedesc);

  return OK;
}

/****************************************************************************
 * Name: fdpic_bind
 *
 * Description:
 *   Apply the module's dynamic relocations.
 *
 *   Both relocation tables are walked, and both eagerly.  Which one an
 *   imported function's descriptor lands in is purely a linker decision --
 *   -z now puts it in DT_REL, and without it the same entry goes to
 *   DT_JMPREL for a lazy resolver to fill in later.  There is no resolver
 *   here, so a table left unwalked is not deferred work, it is a descriptor
 *   that stays unrelocated until the module branches through it.  Binding
 *   both is what makes the layout stop mattering.
 *
 *   Nothing is lost by binding the PLT table early: a module carries a
 *   handful of relocations, so there is no load time worth deferring.
 *
 *   The two tables are not handled identically, though.  A lazy descriptor
 *   arrives pre-loaded with its resolution stub rather than with an addend,
 *   so the PLT table is bound with that word ignored.
 *
 ****************************************************************************/

static int fdpic_bind(FAR struct fdpic_loadinfo_s *loadinfo,
                      FAR struct fdpic_loadinfo_s *head,
                      FAR const struct symtab_s *exports, int nexports)
{
  int ret;

  ret = fdpic_reltable(loadinfo, head, exports, nexports,
                       loadinfo->relvaddr, loadinfo->relsize,
                       false, "DT_REL");
  if (ret < 0)
    {
      return ret;
    }

  return fdpic_reltable(loadinfo, head, exports, nexports,
                        loadinfo->pltrelvaddr, loadinfo->pltrelsize,
                        true, "DT_JMPREL");
}

/****************************************************************************
 * Name: fdpic_callfn
 *
 * Description:
 *   Call a function in a loaded object with that object's data base in the
 *   FDPIC register.
 *
 *   A module's own code normally runs with the register already correct,
 *   because the scheduler installs it from the task's D-Space on every
 *   switch.  Constructors do not get that: they run here, in whichever task
 *   called the loader, before the module's task exists.  So the register has
 *   to be installed by hand around the call, and the function entered
 *   directly rather than through a descriptor.
 *
 *   The base firmware is built with the register reserved, so nothing of the
 *   loader's own is being displaced; saving and restoring it covers the case
 *   where the loader was itself called from module code.  Being preempted
 *   in the middle is harmless -- the register is part of the saved context,
 *   so it travels with whichever task is holding it.
 *
 ****************************************************************************/

#ifdef FDPIC_HAVE_CALLFN
static void fdpic_callfn(uintptr_t entry, uintptr_t got)
{
  __asm__ __volatile__
  (
    "mov r4, r9\n"        /* Save whatever the caller had there  */
    "mov r9, %1\n"        /* This object's data base             */
    "blx %0\n"
    "mov r9, r4\n"
    :
    : "r" (entry), "r" (got)
    : "r0", "r1", "r2", "r3", "r4", "r12", "lr", "cc", "memory"
  );
}
#endif

/****************************************************************************
 * Name: fdpic_callarray
 *
 * Description:
 *   Run one of the DT_INIT_ARRAY / DT_FINI_ARRAY tables of an object.
 *
 *   The array lives in the writable segment and each entry carries an
 *   R_ARM_RELATIVE relocation, so by the time this runs the entries are
 *   already run-time code addresses with their Thumb bit intact.  Nothing
 *   further has to be translated.
 *
 * Input Parameters:
 *   reverse - Walk the array backwards.  Destructors run in the opposite
 *             order to constructors, which is the only reason the two
 *             directions exist.
 *
 ****************************************************************************/

static int fdpic_callarray(FAR struct fdpic_loadinfo_s *obj,
                           uintptr_t vaddr, size_t size, bool reverse,
                           FAR const char *what)
{
  FAR const uintptr_t *array;
#ifdef FDPIC_HAVE_CALLFN
  size_t n;
  size_t i;
#endif

  if (vaddr == 0 || size == 0)
    {
      return OK;
    }

  array = (FAR const uintptr_t *)fdpic_addr(obj, vaddr);
  if (array == NULL)
    {
      berr("ERROR: %s of %s is outside any segment\n", what, obj->name);
      return -ENOEXEC;
    }

#ifndef FDPIC_HAVE_CALLFN
  berr("ERROR: %s has a %s, which this architecture cannot run\n",
       obj->name, what);
  return -ENOSYS;
#else
  n = size / sizeof(uintptr_t);

  binfo("fdpic: %s: %zu entries in %s\n", obj->name, n, what);

  for (i = 0; i < n; i++)
    {
      uintptr_t entry = array[reverse ? n - 1 - i : i];
      uintptr_t code  = entry & ~FDPIC_THUMB_BIT;

      /* 0 and ~0 are the conventional "no function here" fillers */

      if (entry == 0 || entry == (uintptr_t)-1)
        {
          continue;
        }

      /* The entry must point into this object's text.  Left unchecked, a
       * relocation that did not happen -- the failure this loader is most
       * prone to -- becomes a branch to an arbitrary address, which on this
       * class of target means a HardFault with no console and no clue.
       */

      if (code < obj->textaddr || code >= obj->textaddr + obj->textsize)
        {
          berr("ERROR: %s entry %zu of %s is %08lx, outside its text\n",
               what, i, obj->name, (unsigned long)entry);
          return -ENOEXEC;
        }

      fdpic_callfn(entry, obj->gotaddr);
    }

  return OK;
#endif
}

/****************************************************************************
 * Name: fdpic_runinit
 *
 * Description:
 *   Run every object's constructors, dependencies first.
 *
 *   An object joins the list before the DT_NEEDED walk appends its own
 *   dependencies, so a library always sits behind whatever needed it.
 *   Walking the list backwards therefore constructs a library before the
 *   object that uses it, which is the ordering that matters: a module's
 *   constructor may well touch a library object that has to exist already.
 *
 ****************************************************************************/

static int fdpic_runinit(FAR struct fdpic_loadinfo_s *head)
{
  FAR struct fdpic_loadinfo_s *done = NULL;
  FAR struct fdpic_loadinfo_s *obj;
  int ret;

  for (; ; )
    {
      /* The object just ahead of the one done last time.  The list is
       * singly linked and short -- a handful of entries at most -- so
       * rescanning it beats carrying a back pointer around.
       */

      for (obj = head; obj != NULL && obj->flink != done; obj = obj->flink)
        {
        }

      if (obj == NULL)
        {
          return OK;
        }

      ret = fdpic_callarray(obj, obj->initvaddr, obj->initsize, false,
                            "DT_INIT_ARRAY");
      if (ret < 0)
        {
          return ret;
        }

      done = obj;
    }
}

/****************************************************************************
 * Name: fdpic_runfini
 *
 * Description:
 *   Run every object's destructors, in the mirror of the construction
 *   order: the module first, then the libraries it was built on.
 *
 *   Errors are not propagated.  This runs on the teardown path, where there
 *   is nothing left to abandon and reporting a failure would only turn an
 *   incomplete teardown into a leaked one.
 *
 ****************************************************************************/

static void fdpic_runfini(FAR struct fdpic_loadinfo_s *head)
{
  FAR struct fdpic_loadinfo_s *obj;

  for (obj = head; obj != NULL; obj = obj->flink)
    {
      fdpic_callarray(obj, obj->finivaddr, obj->finisize, true,
                      "DT_FINI_ARRAY");
    }
}

/****************************************************************************
 * Name: fdpic_loadbinary
 ****************************************************************************/

static int fdpic_loadbinary(FAR struct binary_s *binp,
                            FAR const char *filename,
                            FAR const struct symtab_s *exports,
                            int nexports)
{
  FAR struct fdpic_loadinfo_s *head = NULL;
  FAR struct fdpic_loadinfo_s *main_obj = NULL;
  FAR struct fdpic_loadinfo_s *obj;
  FAR const char *base;
  int ret;

  binfo("Loading FDPIC module: %s\n", filename);

  base = strrchr(filename, '/');
  base = (base != NULL) ? base + 1 : filename;

  ret = fdpic_loadobject(filename, base, &head, &main_obj);
  if (ret < 0)
    {
      return ret;
    }

  /* Pull in whatever it needs before binding, so that cross-object
   * references have something to resolve against.
   */

  ret = fdpic_loaddepends(main_obj, &head, 0);
  if (ret < 0)
    {
      goto errout;
    }

  /* Bind every object, not just the module.  A library has relocations of
   * its own, and its imports resolve against the same set.
   */

  for (obj = head; obj != NULL; obj = obj->flink)
    {
      ret = fdpic_bind(obj, head, exports, nexports);
      if (ret < 0)
        {
          berr("ERROR: Failed to bind %s: %d\n", obj->name, ret);
          goto errout;
        }
    }

  /* Constructors, now that every object can reach its own data.
   *
   * fdpic_release() can undo a load but cannot undo a constructor, so this
   * sits ahead of the D-Space allocation rather than after it: the only way
   * left to discard a load with constructors already run is a constructor
   * list that fails partway through, and such a module is unusable anyway.
   * Destructors are not run for the entries that did succeed -- a table this
   * loader has just found malformed is not one to branch into on the way
   * out.
   */

  ret = fdpic_runinit(head);
  if (ret < 0)
    {
      berr("ERROR: Constructors of %s failed: %d\n", main_obj->name, ret);
      goto errout;
    }

  binp->entrypt   = (main_t)main_obj->entry;
  binp->mapsize   = 0;
  binp->stacksize = CONFIG_FDPIC_STACKSIZE;
  binp->unload    = fdpic_unloadbinary;

#ifdef CONFIG_PIC
  /* The scheduler expects a reference counted container here, not a bare
   * address: up_initial_state() installs dspace->region into the FDPIC
   * register.  Threads of the task share it via crefs, and
   * sched_releasetcb() frees the container -- but not the region, which is
   * this loader's to release.
   *
   * The module's own GOT goes in.  A call into a library switches the
   * register to that library's GOT via the descriptor, and switches back
   * on return.
   */

  main_obj->dspace = kmm_malloc(sizeof(struct dspace_s));
  if (main_obj->dspace == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  main_obj->dspace->crefs  = 1;
  main_obj->dspace->region = (FAR uint8_t *)main_obj->gotaddr;

  binp->picbase = main_obj->dspace;
#endif

  binp->mapped = head;
  return OK;

errout:
  fdpic_release(head);
  return ret;
}

/****************************************************************************
 * Name: fdpic_unloadbinary
 *
 * Description:
 *   Release the module.  Destructors run first, while the writable segment
 *   they operate on is still there.  Unmapping the text then drops the
 *   filesystem's pin on it, which is what allows a compacting filesystem to
 *   move those blocks again once no instance is executing from them.
 *
 ****************************************************************************/

static int fdpic_unloadbinary(FAR struct binary_s *binp)
{
  fdpic_runfini(binp->mapped);
  fdpic_release(binp->mapped);
  binp->mapped = NULL;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdpic_initialize
 ****************************************************************************/

int fdpic_initialize(void)
{
  int ret;

  ret = register_binfmt(&g_fdpicbinfmt);
  if (ret < 0)
    {
      berr("ERROR: Failed to register FDPIC binfmt: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: fdpic_uninitialize
 ****************************************************************************/

void fdpic_uninitialize(void)
{
  unregister_binfmt(&g_fdpicbinfmt);
}

#endif /* CONFIG_FDPIC */
