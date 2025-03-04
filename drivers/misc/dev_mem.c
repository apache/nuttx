/****************************************************************************
 * drivers/misc/dev_mem.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/drivers/drivers.h>
#include <nuttx/kmalloc.h>
#include <nuttx/memoryregion.h>
#include <sys/param.h>
#include <sys/mman.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_BOARD_MEMORY_RANGE
static const struct memory_region_s g_memory_region[] =
  {
    CONFIG_BOARD_MEMORY_RANGE
  };
#else
extern uint8_t _stext[];           /* Start of .text */
extern uint8_t _etext[];           /* End_1 of .text + .rodata */
extern uint8_t _sdata[];           /* Start of .data */
extern uint8_t _edata[];           /* End+1 of .data */
extern uint8_t _sbss[];            /* Start of .bss */
extern uint8_t _ebss[];            /* End+1 of .bss */

static const struct memory_region_s g_memory_region[] =
  {
    { (uintptr_t)_stext, (uintptr_t)_etext, PROT_EXEC | PROT_READ },
    { (uintptr_t)_sdata, (uintptr_t)_edata, PROT_WRITE | PROT_READ },
    { (uintptr_t)_sbss,  (uintptr_t)_ebss,  PROT_WRITE | PROT_READ },
    { 0, 0, 0 },
  };
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t devmem_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t devmem_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     devmem_mmap(FAR struct file *filep,
                           FAR struct mm_map_entry_s *map);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_devmem_fops =
{
  NULL,                  /* open */
  NULL,                  /* close */
  devmem_read,           /* read */
  devmem_write,          /* write */
  NULL,                  /* seek */
  NULL,                  /* ioctl */
  devmem_mmap,           /* mmap */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devmem_read
 ****************************************************************************/

static ssize_t devmem_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct memory_region_s *region = filep->f_inode->i_private;
  uintptr_t src = filep->f_pos;
  uintptr_t start;
  uintptr_t end;
  ssize_t len;
  int i;

  for (i = 0; region[i].start != 0 && region[i].end != 0; i++)
    {
      start = MAX(src, region[i].start);
      end = MIN(start + buflen, region[i].end);
      len = end - start;
      if (len > 0 && (region[i].flags & PROT_READ))
        {
          memcpy(buffer, (FAR const void *)start, len);
          filep->f_pos = end;
          return len;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: devmem_write
 ****************************************************************************/

static ssize_t devmem_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct memory_region_s *region = filep->f_inode->i_private;
  uintptr_t dest = filep->f_pos;
  uintptr_t start;
  uintptr_t end;
  ssize_t len;
  int i;

  for (i = 0; region[i].start != 0 && region[i].end != 0; i++)
    {
      start = MAX(dest, region[i].start);
      end = MIN(start + buflen, region[i].end);
      len = end - start;
      if (len > 0 && (region[i].flags & PROT_WRITE))
        {
          memcpy((FAR void *)start, buffer, len);
          filep->f_pos = end;
          return len;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: devmem_mmap
 ****************************************************************************/

static int devmem_mmap(FAR struct file *filep,
                       FAR struct mm_map_entry_s *map)
{
  FAR struct memory_region_s *region = filep->f_inode->i_private;
  uintptr_t start;
  uintptr_t end;
  int i;

  if (map->offset < 0)
    {
      return -EINVAL;
    }

  start = map->offset;
  end = start + map->length;

  for (i = 0; region[i].start != 0 && region[i].end != 0; i++)
    {
      if (start >= region[i].start && end <= region[i].end)
        {
          map->vaddr = (FAR void *)start;
          return 0;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devmem_register
 *
 * Description:
 *   Create an MEM driver.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

int devmem_register(void)
{
  return register_driver("/dev/mem", &g_devmem_fops,
                         0666, (FAR void *)g_memory_region);
}
