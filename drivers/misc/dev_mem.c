/****************************************************************************
 * drivers/misc/dev_mem.c
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
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVMEM_REGION 8

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint8_t _stext[];           /* Start of .text */
extern uint8_t _etext[];           /* End_1 of .text + .rodata */
extern uint8_t _sdata[];           /* Start of .data */
extern uint8_t _edata[];           /* End+1 of .data */
extern uint8_t _sbss[];            /* Start of .bss */
extern uint8_t _ebss[];            /* End+1 of .bss */

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

  DEBUGASSERT(region && src);

  for (i = 0; i < DEVMEM_REGION; i++)
    {
      if (region[i].start == 0 && region[i].end == 0)
        {
          break;
        }

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

  DEBUGASSERT(region && dest);

  for (i = 0; i < DEVMEM_REGION; i++)
    {
      if (region[i].start == 0 && region[i].end == 0)
        {
          break;
        }

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

  DEBUGASSERT(region);

  if (map->offset < 0)
    {
      return -EINVAL;
    }

  start = map->offset;
  end = start + map->length;

  for (i = 0; i < DEVMEM_REGION; i++)
    {
      if (region[i].start == 0 && region[i].end == 0)
        {
          break;
        }

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
  FAR struct memory_region_s *region;
  bool merge = (_edata == _sbss);
  ssize_t len = 0;
  int ret;

  region = kmm_calloc(DEVMEM_REGION, sizeof(*region));
  if (region == NULL)
    {
      return -ENOMEM;
    }

  if (CONFIG_BOARD_MEMORY_RANGE[0] != '\0')
    {
      len = parse_memory_region(CONFIG_BOARD_MEMORY_RANGE, region,
                                DEVMEM_REGION - 1);
      if (len < 0)
        {
          kmm_free(region);
          return len;
        }
    }
  else
    {
      if (len + (4 - merge) > DEVMEM_REGION)
        {
          len = DEVMEM_REGION - (4 - merge);
        }

      region[len].flags = PROT_EXEC | PROT_READ;
      region[len].start = (uintptr_t)_stext;
      region[len++].end = (uintptr_t)_etext;
      region[len].flags = PROT_WRITE | PROT_READ;
      region[len].start = (uintptr_t)_sdata;
      region[len++].end = (uintptr_t)_edata;

      if (merge)
        {
          region[len - 1].end = (uintptr_t)_ebss;
        }
      else
        {
          region[len].flags = PROT_WRITE | PROT_READ;
          region[len].start = (uintptr_t)_sbss;
          region[len++].end = (uintptr_t)_ebss;
        }
    }

  /* register the new MEM driver */

  ret = register_driver("/dev/mem", &g_devmem_fops, 0666, region);
  if (ret < 0)
    {
      kmm_free(region);
      return -ENOMEM;
    }

  return ret;
}
