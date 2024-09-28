/****************************************************************************
 * drivers/pci/pci_epc_mem.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <strings.h>
#include <sys/param.h>

#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>
#include <nuttx/lib/math32.h>
#include <nuttx/pci/pci_epc.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_epc_mem_find
 *
 * Description:
 *   Get the matching memory window
 *
 * Input Parameters:
 *   epc       - The EPC device
 *   phys_addr - Virtual address alloced to be matched
 *
 * Returned Value:
 *   Memory window alloced if success, NULL if failed
 ****************************************************************************/

static FAR struct pci_epc_mem_s *
pci_epc_mem_find(FAR struct pci_epc_ctrl_s *epc,
                 uintptr_t phys_addr)
{
  unsigned int i;

  for (i = 0; i < epc->num_windows; i++)
    {
      if (phys_addr >= epc->mem[i].phys_base &&
          phys_addr < epc->mem[i].phys_base + epc->mem[i].size)
        {
          return &epc->mem[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_epc_mem_multi_init
 *
 * Description:
 *   This function is used to initialize the pci_epc_mem_s structure.
 *
 *   Invoke to initialize the pci_epc_mem_s structure used by the
 * endpoint functions to allocate mapped PCI address.
 *
 * Input Parameters:
 *   epc         - The EPC device that invoked pci_epc_mem_init
 *   windows     - Pointer to windows supported by the device
 *   num_windows - Number of windows device supports
 *
 * Returned Value:
 *   0 if success, negative if failed
 ****************************************************************************/

int pci_epc_mem_multi_init(FAR struct pci_epc_ctrl_s *epc,
                           FAR const struct pci_epc_mem_window_s *windows,
                           unsigned int num_windows)
{
  unsigned int i;

  if (epc == NULL || windows == NULL)
    {
      return -EINVAL;
    }

  epc->mem = kmm_calloc(num_windows, sizeof(*epc->mem));
  if (epc->mem == NULL)
    {
      return -ENOMEM;
    }

  for (i = 0; i < num_windows; i++)
    {
      size_t pages = windows[i].size / windows[i].page_size;
      size_t bitmap_size = BITS_TO_LONGS(pages) * sizeof(long);

      FAR unsigned long *bitmap = kmm_zalloc(bitmap_size);
      if (bitmap == NULL)
        {
          goto err;
        }

      epc->mem[i].virt_base = windows[i].virt_base;
      epc->mem[i].phys_base = windows[i].phys_base;
      epc->mem[i].size = windows[i].size;
      epc->mem[i].page_size = windows[i].page_size;
      epc->mem[i].bitmap = bitmap;
      epc->mem[i].pages = pages;
      nxmutex_init(&epc->mem[i].lock);
    }

  epc->num_windows = num_windows;
  return 0;

err:
  while (i-- > 0)
    {
      nxmutex_destroy(&epc->mem[i].lock);
      kmm_free(epc->mem[i].bitmap);
    }

  kmm_free(epc->mem);
  epc->mem = NULL;

  return -ENOMEM;
}

/****************************************************************************
 * Name: pci_epc_mem_init
 *
 * Description:
 *   This function is used to initialize the PCI endpoint controller memory
 * space.
 *
 * Input Parameters:
 *   epc       - PCI EPC device
 *   virt      - The virtual base address of the PCI address window
 *   phys      - The phys base address of the PCI address window
 *   size      - The PCI window size
 *   page_size - Size of each window page
 *
 * Returned Value:
 *   0 if success, negative if failed
 ****************************************************************************/

int pci_epc_mem_init(FAR struct pci_epc_ctrl_s *epc, FAR void *virt,
                     uintptr_t phys, size_t size, size_t page_size)
{
  struct pci_epc_mem_window_s window;

  window.virt_base = virt;
  window.phys_base = phys;
  window.size = size;
  window.page_size = page_size;

  return pci_epc_mem_multi_init(epc, &window, 1);
}

/****************************************************************************
 * Name: pci_epc_mem_exit
 *
 * Description:
 *   This function is used to cleanup the pci_epc_mem_s structure.
 *
 * Invoke to cleanup the pci_epc_mem_s structure allocated in
 * pci_epc_mem_init().
 *
 * Input Parameters:
 *   epc - EPC device that invoked pci_epc_mem_exit
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epc_mem_exit(FAR struct pci_epc_ctrl_s *epc)
{
  unsigned int i;

  if (epc->num_windows == 0)
    {
      return;
    }

  for (i = 0; i < epc->num_windows; i++)
    {
      nxmutex_destroy(&epc->mem[i].lock);
      kmm_free(epc->mem[i].bitmap);
    }

  kmm_free(epc->mem);
  epc->mem = NULL;
  epc->num_windows = 0;
}

/****************************************************************************
 * Name: pci_epc_mem_alloc_addr
 *
 * Description:
 *   Allocate memory address from EPC addr space
 *
 *   Invoke to allocate memory address from the EPC address space. This
 * is usually done to map the remote RC address into the local system.
 *
 * Input Parameters:
 *   epc   - The EPC device on which memory has to be allocated
 *   phys  - The virtual addr
 *   size  - The size of the address space that has to be allocated
 *
 * Returned Value:
 *   The memory address alloced if success, NULL if failed
 ****************************************************************************/

FAR void *pci_epc_mem_alloc_addr(FAR struct pci_epc_ctrl_s *epc,
                                 FAR uintptr_t *phys, size_t size)
{
  unsigned int i;

  for (i = 0; i < epc->num_windows; i++)
    {
      FAR struct pci_epc_mem_s *mem = &epc->mem[i];
      size_t pages = div_round_up(size, mem->page_size);
      size_t pageno;

      nxmutex_lock(&mem->lock);
      pageno = bitmap_find_free_region(mem->bitmap, mem->pages, pages);
      nxmutex_unlock(&mem->lock);

      if (pageno != mem->pages)
        {
          *phys =  mem->phys_base + pageno * mem->page_size;
          return mem->virt_base + pageno * mem->page_size;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: pci_epc_mem_free_addr
 *
 * Description:
 *   Free the allocated memory address.
 *
 *   Invoke to free the memory allocated using pci_epc_mem_alloc_addr.
 *
 * Input Parameters:
 *   epc       - The EPC device on which memory was allocated
 *   phys_addr - The allocated virtual address
 *   size      - The size of the allocated address space
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void pci_epc_mem_free_addr(FAR struct pci_epc_ctrl_s *epc,
                           uintptr_t phys_addr, size_t size)
{
  FAR struct pci_epc_mem_s *mem;
  uintptr_t pageno;
  size_t pages;

  mem = pci_epc_mem_find(epc, phys_addr);
  if (mem == NULL)
    {
      pcierr("Failed to get matching window\n");
      return;
    }

  pageno = (phys_addr - mem->phys_base) / mem->page_size;
  pages = div_round_up(size, mem->page_size);

  nxmutex_lock(&mem->lock);
  bitmap_release_region(mem->bitmap, pageno, pages);
  nxmutex_unlock(&mem->lock);
}

