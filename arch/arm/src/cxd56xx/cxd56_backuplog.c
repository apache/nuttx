/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_backuplog.c
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

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <arch/chip/backuplog.h>
#include "chip.h"
#include "hardware/cxd5602_backupmem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Log header information */

#define CXD56_LOG_MAGIC         (0x48474F4C) /* 'L''O''G''H' */
#define CXD56_LOG_ENTRY_NUM     (31)
#define CXD56_LOG_ENTRY_NAME    (8)

/* Log header magic check */

#define CXD56_LOG_MAGIC_CHECK(h) do { \
  if ((h)->magic != CXD56_LOG_MAGIC) \
    { \
      up_backuplog_initialize(); \
    } \
} while (0)

/* Log region area definitions */

#define AREASIZE                (512)
#define AREASHIFT               (9)
#define AREAMASK                (0x000001ff)

/* Alignment macros */

#define AREA_ALIGNDOWN(size)    ((size) & ~AREAMASK)
#define AREA_ALIGNUP(size)      (((size) + AREAMASK) & ~AREAMASK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct entry_s
{
  char name[CXD56_LOG_ENTRY_NAME];
  void *addr;
  size_t size;
};

struct logheader_s
{
  uint32_t magic;
  uint32_t flags;
  uint64_t used;
  struct entry_s entry[CXD56_LOG_ENTRY_NUM];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct logheader_s *get_logheader(void)
{
  return (struct logheader_s *)BKUP->log;
}

static int search_entry(const char *name)
{
  struct logheader_s *header = get_logheader();
  int i;

  for (i = 0; i < CXD56_LOG_ENTRY_NUM; i++)
    {
      if (0 == strncmp(header->entry[i].name, name, CXD56_LOG_ENTRY_NAME))
        {
          return i;
        }
    }

  return -ENOENT;
}

static void *get_allocated_memory(const char *name, size_t size)
{
  int       index;
  void      *memaddr;
  size_t    memsize;

  index = up_backuplog_region(name, &memaddr, &memsize);
  if (index >= 0)
    {
      /* Already allocated */

      if (memsize == size)
        {
          return memaddr;
        }
      else
        {
          /* If size is different, free here and re-allocate later */

          up_backuplog_free(name);
        }
    }

  return NULL;
}

static int allocate_memory(size_t size)
{
  struct logheader_s *header = get_logheader();
  int       allocated = 0;
  int       alloc_num;
  uint64_t  alloc_bits;
  int       i;

  alloc_num = AREA_ALIGNUP(size) >> AREASHIFT;
  alloc_bits = (1ULL << alloc_num) - 1;
  for (i = 0; i <= 64 - alloc_num; i++)
    {
      if ((~header->used & alloc_bits) == alloc_bits)
        {
          allocated = i;
          break;
        }

      alloc_bits <<= 1;
    }

  if (allocated)
    {
      header->used |= alloc_bits;
    }

  return allocated;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_backuplog_initialize
 *
 * Description:
 *   Initialize the log header where the address and size of each log area
 *   are described. If the log header has been already configured in a wakeup
 *   from sleeping or reboot case, then do nothing and return OK.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int up_backuplog_initialize(void)
{
  struct logheader_s *header = get_logheader();

  if (header->magic != CXD56_LOG_MAGIC)
    {
      /* Initialize log header */

      memset(header, 0, sizeof(struct logheader_s));

      header->magic = CXD56_LOG_MAGIC;
      header->flags = 0;
      header->used = 0x1;
    }

  return 0;
}

/****************************************************************************
 * Name: up_backuplog_alloc
 *
 * Description:
 *   Allocate the log memory region
 *
 * Input Parameters:
 *   name - The log region name
 *   size - The size to allocate
 *
 * Returned Value:
 *   The allocated address on success; NULL value on failure.
 *
 ****************************************************************************/

void *up_backuplog_alloc(const char *name, size_t size)
{
  struct logheader_s *header = get_logheader();
  irqstate_t flags;
  int       index;
  int       allocated = 0;
  void      *addr = NULL;

  if (0 == size)
    {
      return NULL;
    }

  flags = enter_critical_section();

  CXD56_LOG_MAGIC_CHECK(header);

  /* Already allocated or not */

  addr = get_allocated_memory(name, size);
  if (addr)
    {
      goto exit;
    }

  /* Allocate a new memory region */

  allocated = allocate_memory(size);
  if (0 == allocated)
    {
      goto exit;
    }

  /* Register a entry into empty region */

  for (index = 0; index < CXD56_LOG_ENTRY_NUM; index++)
    {
      if (0 == header->entry[index].size)
        {
          strncpy(header->entry[index].name, name, CXD56_LOG_ENTRY_NAME);
          header->entry[index].addr = (void *)((uint32_t)header +
                                               (allocated << AREASHIFT));
          header->entry[index].size = size;

          addr = header->entry[index].addr;
          break;
        }
    }

exit:
  leave_critical_section(flags);
  return addr;
}

/****************************************************************************
 * Name: up_backuplog_free
 *
 * Description:
 *   De-allocate the log memory region
 *
 * Input Parameters:
 *   name - The log region name
 *
 ****************************************************************************/

void up_backuplog_free(const char *name)
{
  struct logheader_s *header = get_logheader();
  irqstate_t flags;
  int       index;
  void      *addr;
  size_t    size;
  uint64_t  alloc_bits;
  int       offset;

  flags = enter_critical_section();

  CXD56_LOG_MAGIC_CHECK(header);

  index = up_backuplog_region(name, &addr, &size);
  if (index >= 0)
    {
      alloc_bits = (1ULL << (AREA_ALIGNUP(size) >> AREASHIFT)) - 1;
      offset = ((uint32_t)addr - (uint32_t)header) >> AREASHIFT;
      alloc_bits <<= offset;

      header->used &= ~alloc_bits;
      memset(&header->entry[index], 0, sizeof(struct entry_s));
    }

  leave_critical_section(flags);

  return;
}

/****************************************************************************
 * Name: up_backuplog_region
 *
 * Description:
 *   Get the address and size of the specified log region name
 *
 * Input Parameters:
 *   name - The log region name
 *   addr - The returned address
 *   size - The returned size
 *
 * Returned Value:
 *   The index of log entry on success; A negated errno value on failure.
 *
 ****************************************************************************/

int up_backuplog_region(const char *name, void **addr, size_t *size)
{
  struct logheader_s *header = get_logheader();
  irqstate_t flags;
  int index;

  DEBUGASSERT(addr);
  DEBUGASSERT(size);

  *addr = NULL;
  *size = 0;

  flags = enter_critical_section();

  CXD56_LOG_MAGIC_CHECK(header);

  index = search_entry(name);
  if (index >= 0)
    {
      *addr = header->entry[index].addr;
      *size = header->entry[index].size;
    }

  leave_critical_section(flags);

  return index;
}

/****************************************************************************
 * Name: up_backuplog_entry
 *
 * Description:
 *   Get the entry name, address and size
 *
 * Input Parameters:
 *   name - The returned entry name
 *   addr - The returned address
 *   size - The returned size
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int up_backuplog_entry(char *name, void **addr, size_t *size)
{
  struct logheader_s *header = get_logheader();
  irqstate_t flags;
  int index;

  DEBUGASSERT(name);
  DEBUGASSERT(addr);
  DEBUGASSERT(size);

  *addr = NULL;
  *size = 0;

  flags = enter_critical_section();

  CXD56_LOG_MAGIC_CHECK(header);

  for (index = 0; index < CXD56_LOG_ENTRY_NUM; index++)
    {
      if ('\0' != header->entry[index].name[0])
        {
          break;
        }
    }

  leave_critical_section(flags);

  if (index < CXD56_LOG_ENTRY_NUM)
    {
      memcpy(name, header->entry[index].name, CXD56_LOG_ENTRY_NAME);
      *addr = header->entry[index].addr;
      *size = header->entry[index].size;
      return 0;
    }
  else
    {
      return -ENOENT;
    }
}
