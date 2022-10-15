/****************************************************************************
 * arch/xtensa/src/esp32/esp32_himem.c
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

#include <stdlib.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/himem/himem.h>
#include <nuttx/spinlock.h>

#include "esp32_spiram.h"
#include "esp32_himem.h"
#include "hardware/esp32_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* So, why does the API look this way and is so inflexible to not allow any
 * maps beyond the full 32K chunks? Most of it has to do with the fact that
 * the cache works on the *virtual* addresses What this comes down to is that
 * while it's allowed to map a range of physical memory into the address
 * space two times, there's no cache consistency between the two regions.
 *
 * This means that a write to region A may or may not show up, perhaps
 * delayed, in region B, as it depends on the time that the writeback to SPI
 * RAM is done on A and the time before the corresponding cache line is
 * invalidated on B. Note that this goes for every 32-byte cache line: this
 * implies that if a program writes to address X and Y within A, the write to
 * Y may show up before the write to X does.
 *
 * It gets even worse when both A and B are written: theoretically, a write
 * to a 32-byte cache line in A can be entirely undone because of a write to
 * a different address in B that happens to be in the same 32-byte cache
 * line.
 *
 * Because of these reasons, we do not allow double mappings at all. This,
 * however, has other implications that make supporting ranges not really
 * useful. Because the lack of double mappings, applications will need to do
 * their own management of mapped regions, meaning they will normally map in
 * and out blocks at a time anyway, as mapping more fluent regions would
 * result in the chance of accidentally mapping two overlapping regions. As
 * this is the case, to keep the code simple, at the moment we just force
 * these blocks to be equal to the 32K MMU page size. The API itself does
 * allow for more granular allocations, so if there's a pressing need for a
 * more complex solution in the future, we can do this.
 *
 * Note: In the future, we can expand on this api to do a memcpy() between
 * SPI RAM and (internal) memory using the SPI1 peripheral. This needs
 * support for SPI1 to be in the SPI driver, however.
 */

/* How many 32KB pages will be reserved for bank switch */

#ifdef CONFIG_ESP32_SPIRAM_BANKSWITCH_ENABLE
#  define SPIRAM_BANKSWITCH_RESERVE CONFIG_SPIRAM_BANKSWITCH_RESERVE
#else
#  define SPIRAM_BANKSWITCH_RESERVE 0
#endif

#define CACHE_BLOCKSIZE (32*1024)

/* Start of the virtual address range reserved for himem use */

#define VIRT_HIMEM_RANGE_START (SOC_EXTRAM_DATA_LOW + \
                                (128 - SPIRAM_BANKSWITCH_RESERVE) * \
                                CACHE_BLOCKSIZE)

/* Start MMU block reserved for himem use */

#define VIRT_HIMEM_RANGE_BLOCKSTART (128-SPIRAM_BANKSWITCH_RESERVE)

/* Start physical block */

#define PHYS_HIMEM_BLOCKSTART (128 - SPIRAM_BANKSWITCH_RESERVE)

#define HIMEM_CHECK(cond, str, err) if (cond) \
                                       do \
                                         { merr("%s: %s", __FUNCTION__, str); \
                                           return err; \
                                         } while(0)

/* Character driver methods */

static ssize_t himem_read(struct file *filep, char *buffer,
                          size_t buflen);
static ssize_t himem_write(struct file *filep, const char *buffer,
                           size_t buflen);
static int     himem_ioctl(struct file *filep, int cmd,
                           unsigned long arg);

/* This structure is used only for access control */

struct himem_access_s
{
  sem_t        exclsem;  /* Supports mutual exclusion */
};

/* Metadata for a block of physical RAM */

typedef struct
{
  unsigned int is_alloced: 1;
  unsigned int is_mapped: 1;
} ramblock_t;

/* Metadata for a 32-K memory address range */

typedef struct
{
  unsigned int is_alloced: 1;
  unsigned int is_mapped: 1;
  unsigned int ram_block: 16;
} rangeblock_t;

static ramblock_t *g_ram_descriptor = NULL;
static rangeblock_t *g_range_descriptor = NULL;
static int g_ramblockcnt = 0;
static const int g_rangeblockcnt = SPIRAM_BANKSWITCH_RESERVE;

/* Used by the spinlock */

irqstate_t spinlock_flags;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_himemfops =
{
  NULL,             /* open   */
  NULL,             /* close */
  himem_read,       /* read */
  himem_write,      /* write */
  NULL,             /* seek */
  himem_ioctl,      /* ioctl */
  NULL              /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

static inline int ramblock_idx_valid(int ramblock_idx)
{
  return (ramblock_idx >= 0 && ramblock_idx < g_ramblockcnt);
}

static inline int rangeblock_idx_valid(int rangeblock_idx)
{
  return (rangeblock_idx >= 0 && rangeblock_idx < g_rangeblockcnt);
}

static void set_bank(int virt_bank, int phys_bank, int ct)
{
  int r;

  r = cache_sram_mmu_set(0, 0, SOC_EXTRAM_DATA_LOW + CACHE_BLOCKSIZE *
                         virt_bank, phys_bank * CACHE_BLOCKSIZE, 32, ct);
  DEBUGASSERT(r == 0);
  r = cache_sram_mmu_set(1, 0, SOC_EXTRAM_DATA_LOW + CACHE_BLOCKSIZE *
                         virt_bank, phys_bank * CACHE_BLOCKSIZE, 32, ct);
  DEBUGASSERT(r == 0);

  UNUSED(r);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t esp_himem_get_phys_size(void)
{
  int paddr_start = (4096 * 1024) - (CACHE_BLOCKSIZE *
                                     SPIRAM_BANKSWITCH_RESERVE);
  return esp_spiram_get_size() - paddr_start;
}

size_t esp_himem_get_free_size(void)
{
  size_t ret = 0;
  int i;

  for (i = 0; i < g_ramblockcnt; i++)
    {
      if (!g_ram_descriptor[i].is_alloced)
        {
          ret += CACHE_BLOCKSIZE;
        }
    }

  return ret;
}

size_t esp_himem_reserved_area_size(void)
{
  return CACHE_BLOCKSIZE * SPIRAM_BANKSWITCH_RESERVE;
}

int esp_himem_init(void)
{
  struct himem_access_s *priv;
  int paddr_start = (4096 * 1024) - (CACHE_BLOCKSIZE *
                     SPIRAM_BANKSWITCH_RESERVE);
  int paddr_end;
  int maxram;
  int ret;

  if (SPIRAM_BANKSWITCH_RESERVE == 0)
    {
      return -ENODEV;
    }

  /* Allocate a new himem access instance */

  priv = (struct himem_access_s *)
    kmm_zalloc(sizeof(struct himem_access_s));

  if (!priv)
    {
      merr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  maxram = esp_spiram_get_size();

  /* Catch double init */

  /* Looks weird; last arg is empty so it expands to 'return ;' */

  HIMEM_CHECK(g_ram_descriptor != NULL, "already initialized", 0);

  HIMEM_CHECK(g_range_descriptor != NULL, "already initialized", 0);

  /* need to have some reserved banks */

  HIMEM_CHECK(SPIRAM_BANKSWITCH_RESERVE == 0, "No banks reserved for \
              himem", 0);

  /* Start and end of physical reserved memory. Note it starts slightly under
   * the 4MiB mark as the reserved banks can't have an unity mapping to be
   * used by malloc anymore; we treat them as himem instead.
   */

  paddr_end = maxram;
  g_ramblockcnt = ((paddr_end - paddr_start) / CACHE_BLOCKSIZE);

  /* Allocate data structures */

  g_ram_descriptor = kmm_zalloc(sizeof(ramblock_t) * g_ramblockcnt);
  g_range_descriptor = kmm_zalloc(sizeof(rangeblock_t) * \
                              SPIRAM_BANKSWITCH_RESERVE);

  if (g_ram_descriptor == NULL || g_range_descriptor == NULL)
    {
      merr("Cannot allocate memory for meta info. Not initializing!");
      free(g_ram_descriptor);
      free(g_range_descriptor);
      return -ENOMEM;
    }

  /* Register the character driver */

  ret = register_driver("/dev/himem", &g_himemfops, 0666, priv);
  if (ret < 0)
    {
      merr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  minfo("Initialized. Using last %d 32KB address blocks for bank \
        switching on %d KB of physical memory.\n",
        SPIRAM_BANKSWITCH_RESERVE, (paddr_end - paddr_start) / 1024);

  return OK;
}

/* Allocate count not-necessarily consecutive physical RAM blocks, return
 * numbers in blocks[]. Return true if blocks can be allocated, false if not.
 */

static bool allocate_blocks(int count, uint16_t *blocks_out)
{
  int n = 0;
  int i;

  for (i = 0; i < g_ramblockcnt && n != count; i++)
    {
      if (!g_ram_descriptor[i].is_alloced)
        {
          blocks_out[n] = i;
          n++;
        }
    }

  if (n == count)
    {
      /* All blocks could be allocated. Mark as in use. */

      for (i = 0; i < count; i++)
        {
          g_ram_descriptor[blocks_out[i]].is_alloced = true;
          DEBUGASSERT(g_ram_descriptor[blocks_out[i]].is_mapped  == false);
        }

      return true;
    }
  else
    {
      /* Error allocating blocks */

      return false;
    }
}

int esp_himem_alloc(size_t size, esp_himem_handle_t *handle_out)
{
  esp_himem_ramdata_t *r;
  int blocks;
  int ok;

  /* If the size is not multiple of BLOCKSIZE, there is an issue */

  if (size % CACHE_BLOCKSIZE != 0)
    {
      return -EINVAL;
    }

  blocks = size / CACHE_BLOCKSIZE;

  r = kmm_malloc(sizeof(esp_himem_ramdata_t));
  if (!r)
    {
      goto nomem;
    }

  r->block = kmm_malloc(sizeof(uint16_t) * blocks);
  if (!r->block)
    {
      goto nomem;
    }

  spinlock_flags = spin_lock_irqsave(NULL);

  ok = allocate_blocks(blocks, r->block);

  spin_unlock_irqrestore(NULL, spinlock_flags);
  if (!ok)
    {
      goto nomem;
    }

  r->block_ct = blocks;
  *handle_out = r;

  return OK;

nomem:
  if (r)
    {
      free(r->block);
      free(r);
    }

  return -ENOMEM;
}

int esp_himem_free(esp_himem_handle_t handle)
{
  int i;

  /* Check if any of the blocks is still mapped; fail if this is the case. */

  for (i = 0; i < handle->block_ct; i++)
    {
      DEBUGASSERT(ramblock_idx_valid(handle->block[i]));
      HIMEM_CHECK(g_ram_descriptor[handle->block[i]].is_mapped,
                  "block in range still mapped", -EINVAL);
    }

  /* Mark blocks as free */

  spinlock_flags = spin_lock_irqsave(NULL);
  for (i = 0; i < handle->block_ct; i++)
    {
      g_ram_descriptor[handle->block[i]].is_alloced = false;
    }

  spin_unlock_irqrestore(NULL, spinlock_flags);

  /* Free handle */

  free(handle->block);
  free(handle);
  return OK;
}

int esp_himem_alloc_map_range(size_t size,
                              esp_himem_rangehandle_t *handle_out)
{
  esp_himem_rangedata_t *r;
  int i;
  int blocks;
  int start_free;

  HIMEM_CHECK(g_ram_descriptor == NULL, "Himem not available!",
              -EINVAL);

  HIMEM_CHECK(size % CACHE_BLOCKSIZE != 0,
              "requested size not aligned to blocksize",
              -EINVAL);

  blocks = size / CACHE_BLOCKSIZE;

  r = kmm_malloc(sizeof(esp_himem_rangedata_t) * 1);
  if (!r)
    {
      return -ENOMEM;
    }

  r->block_ct = blocks;
  r->block_start = -1;

  start_free = 0;
  spinlock_flags = spin_lock_irqsave(NULL);

  for (i = 0; i < g_rangeblockcnt; i++)
    {
      if (g_range_descriptor[i].is_alloced)
        {
          start_free = i + 1; /* optimistically assume next block is free... */
        }
      else
        {
          if (i - start_free == blocks - 1)
            {
              /* We found a span of blocks that's big enough to allocate
               * the requested range in.
               */

              r->block_start = start_free;
              break;
            }
        }
    }

  if (r->block_start == -1)
    {
      /* Couldn't find enough free blocks */

      free(r);
      spin_unlock_irqrestore(NULL, spinlock_flags);
      return -ENOMEM;
    }

  /* Range is found. Mark the blocks as in use. */

  for (i = 0; i < blocks; i++)
    {
      g_range_descriptor[r->block_start + i].is_alloced = 1;
    }

  spin_unlock_irqrestore(NULL, spinlock_flags);

  /* All done. */

  *handle_out = r;
  return OK;
}

int esp_himem_free_map_range(esp_himem_rangehandle_t handle)
{
  int i;

  /* Check if any of the blocks in the range have a mapping */

  for (i = 0; i < handle->block_ct; i++)
    {
      DEBUGASSERT(rangeblock_idx_valid(handle->block_start + i));

      /* should be allocated, if handle is valid */

      DEBUGASSERT(g_range_descriptor[i + \
                  handle->block_start].is_alloced == 1);

      HIMEM_CHECK(g_range_descriptor[i + handle->block_start].is_mapped,
                  "memory still mapped to range", -EINVAL);
    }

  /* We should be good to free this. Mark blocks as free. */

  spinlock_flags = spin_lock_irqsave(NULL);

  for (i = 0; i < handle->block_ct; i++)
    {
      g_range_descriptor[i + handle->block_start].is_alloced = 0;
    }

  spin_unlock_irqrestore(NULL, spinlock_flags);
  free(handle);
  return OK;
}

int esp_himem_map(esp_himem_handle_t handle,
                  esp_himem_rangehandle_t range,
                  size_t ram_offset,
                  size_t range_offset,
                  size_t len,
                  int flags,
                  void **out_ptr)
{
  int i;
  int ram_block = ram_offset / CACHE_BLOCKSIZE;
  int range_block = range_offset / CACHE_BLOCKSIZE;
  int blockcount = len / CACHE_BLOCKSIZE;

  HIMEM_CHECK(g_ram_descriptor == NULL, "Himem not available!",
              -EINVAL);

  /* Offsets and length must be block-aligned */

  HIMEM_CHECK(ram_offset % CACHE_BLOCKSIZE != 0,
              "ram offset not aligned to blocksize", -EINVAL);

  HIMEM_CHECK(range_offset % CACHE_BLOCKSIZE != 0,
              "range not aligned to blocksize", -EINVAL);

  HIMEM_CHECK(len % CACHE_BLOCKSIZE != 0,
              "length not aligned to blocksize", -EINVAL);

  /* ram and range should be within allocated range */

  HIMEM_CHECK(ram_block + blockcount > handle->block_ct,
              "args not in range of phys ram handle", -EINVAL);

  HIMEM_CHECK(range_block + blockcount > range->block_ct,
              "args not in range of range handle", -EINVAL);

  /* Check if ram blocks aren't already mapped, and if memory range is
   * unmapped.
   */

  for (i = 0; i < blockcount; i++)
    {
      HIMEM_CHECK(g_ram_descriptor[handle->block[i + ram_block]].is_mapped,
                  "ram already mapped", -EINVAL);

      HIMEM_CHECK(g_range_descriptor[range->block_start + i +
                  range_block].is_mapped, "range already mapped",
                  -EINVAL);
    }

  /* Map and mark as mapped */

  spinlock_flags = spin_lock_irqsave(NULL);

  for (i = 0; i < blockcount; i++)
    {
      DEBUGASSERT(ramblock_idx_valid(handle->block[i + ram_block]));
      g_ram_descriptor[handle->block[i + ram_block]].is_mapped = 1;
      g_range_descriptor[range->block_start + i + range_block].is_mapped = 1;
      g_range_descriptor[range->block_start + i + range_block].ram_block =
                        handle->block[i + ram_block];
    }

  spin_unlock_irqrestore(NULL, spinlock_flags);

  for (i = 0; i < blockcount; i++)
    {
      set_bank(VIRT_HIMEM_RANGE_BLOCKSTART + range->block_start + i +
               range_block, handle->block[i + ram_block] +
               PHYS_HIMEM_BLOCKSTART, 1);
    }

  /* Set out pointer */

  *out_ptr = (void *)(VIRT_HIMEM_RANGE_START +
             (range->block_start + range_offset) * CACHE_BLOCKSIZE);

  return OK;
}

int esp_himem_unmap(esp_himem_rangehandle_t range, void *ptr,
                    size_t len)
{
  /* Note: doesn't actually unmap, just clears cache and marks blocks as
   * unmapped.
   * Future optimization: could actually lazy-unmap here: essentially, do
   * nothing and only clear the cache when we re-use the block for a
   * different physical address.
   */

  int range_offset = (uint32_t)ptr - VIRT_HIMEM_RANGE_START;
  int range_block = (range_offset / CACHE_BLOCKSIZE) - range->block_start;
  int blockcount = len / CACHE_BLOCKSIZE;
  int i;

  HIMEM_CHECK(range_offset % CACHE_BLOCKSIZE != 0,
              "range offset not block-aligned", -EINVAL);

  HIMEM_CHECK(len % CACHE_BLOCKSIZE != 0,
              "map length not block-aligned", -EINVAL);

  HIMEM_CHECK(range_block + blockcount > range->block_ct,
              "range out of bounds for handle", -EINVAL);

  spinlock_flags = spin_lock_irqsave(NULL);

  for (i = 0; i < blockcount; i++)
    {
      int ramblock = g_range_descriptor[range->block_start + i +
                     range_block].ram_block;

      DEBUGASSERT(ramblock_idx_valid(ramblock));
      g_ram_descriptor[ramblock].is_mapped = 0;
      g_range_descriptor[range->block_start + i + range_block].is_mapped = 0;
    }

  esp_spiram_writeback_cache();
  spin_unlock_irqrestore(NULL, spinlock_flags);
  return OK;
}

/****************************************************************************
 * Name: himem_read
 ****************************************************************************/

static ssize_t himem_read(struct file *filep, char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: himem_write
 ****************************************************************************/

static ssize_t himem_write(struct file *filep, const char *buffer,
                          size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: himem_ioctl
 ****************************************************************************/

static int himem_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      /* Allocate the physical RAM blocks */

      case HIMEMIOC_ALLOC_BLOCKS:
        {
          struct esp_himem_par *param =
                     (struct esp_himem_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          /* Allocate the memory we're going to check. */

          ret = esp_himem_alloc(param->memfree, &(param->handle));
          if (ret < 0)
            {
              minfo("Error: esp_himem_alloc() failed!\n");
              return ret;
            }
        }
        break;

      /* Free the physical RAM blocks */

      case HIMEMIOC_FREE_BLOCKS:
        {
          struct esp_himem_par *param =
                     (struct esp_himem_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          ret = esp_himem_free(param->handle);
          if (ret < 0)
            {
              minfo("Error: esp_himem_free() failed!\n");
              return ret;
            }
        }
        break;

      /* Allocate the mapping range */

      case HIMEMIOC_ALLOC_MAP_RANGE:
        {
          struct esp_himem_par *param =
                     (struct esp_himem_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          /* Allocate a block of address range */

          ret = esp_himem_alloc_map_range(ESP_HIMEM_BLKSZ, &(param->range));
          if (ret < 0)
            {
              minfo("Error: esp_himem_alloc_map_range() failed!\n");
              return ret;
            }
        }
        break;

      /* Free the mapping range */

      case HIMEMIOC_FREE_MAP_RANGE:
        {
          struct esp_himem_par *param =
                     (struct esp_himem_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          ret = esp_himem_free_map_range(param->range);
          if (ret < 0)
            {
              minfo("Error: esp_himem_free_map_range() failed!\n");
              return ret;
            }
        }
        break;

      /* Map the himem blocks */

      case HIMEMIOC_MAP:
        {
          struct esp_himem_par *param =
                     (struct esp_himem_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          ret = esp_himem_map(param->handle,
                              param->range,
                              param->ram_offset,
                              param->range_offset,
                              param->len,
                              param->flags,
                              (void **) &(param->ptr));
          if (ret < 0)
            {
              minfo("error: esp_himem_map() failed!\n");
              return ret;
            }
        }
        break;

      /* Unmap the himem blocks */

      case HIMEMIOC_UNMAP:
        {
          struct esp_himem_par *param =
                     (struct esp_himem_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          ret = esp_himem_unmap(param->range,
                                (void *) param->ptr,
                                param->len);
          if (ret < 0)
            {
              minfo("error: esp_himem_unmap() failed!\n");
              return ret;
            }
        }
        break;

      /* Get the physical external memory size */

      case HIMEMIOC_GET_PHYS_SIZE:
        {
          struct esp_himem_par *param =
                     (struct esp_himem_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          param->memcnt = esp_himem_get_phys_size();
        }
        break;

      /* Get the free memory size */

      case HIMEMIOC_GET_FREE_SIZE:
        {
          struct esp_himem_par *param =
                     (struct esp_himem_par *)((uintptr_t)arg);

          DEBUGASSERT(param != NULL);

          param->memfree = esp_himem_get_free_size();
        }
        break;

      default:
        {
          sninfo("Unrecognized cmd: %d\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

