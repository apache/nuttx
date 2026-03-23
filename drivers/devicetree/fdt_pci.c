/****************************************************************************
 * drivers/devicetree/fdt_pci.c
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

#include <errno.h>

#include <nuttx/pci/pci_ecam.h>
#include <nuttx/fdt.h>

#include <libfdt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FDT_PCI_TYPE_IO       0x01000000
#define FDT_PCI_TYPE_MEM32    0x02000000
#define FDT_PCI_TYPE_MEM64    0x03000000
#define FDT_PCI_TYPE_MASK     0x03000000
#define FDT_PCI_PREFETCH      0x40000000

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_pci_ecam_register
 *
 * Description:
 *   This function is used to register an ecam driver from the device tree
 *
 * Input Parameters:
 *   fdt      - Device tree handle
 *
 * Returned Value:
 *   Return 0 if success, nageative if failed
 *
 ****************************************************************************/

int fdt_pci_ecam_register(FAR const void *fdt)
{
  struct pci_resource_s prefetch;
  struct pci_resource_s cfg;
  struct pci_resource_s mem;
  struct pci_resource_s io;
  FAR const fdt32_t *ranges;
  int offset;

  /* #address-size must be 3
   * defined in the PCI Bus Binding to IEEE Std 1275-1994 :
   * Bit#
   *
   * phys.hi cell:  npt000ss bbbbbbbb dddddfff rrrrrrrr
   * phys.mid cell: hhhhhhhh hhhhhhhh hhhhhhhh hhhhhhhh
   * phys.lo cell:  llllllll llllllll llllllll llllllll
   */

  const int na = 3;

  /* #size-cells must be 2 */

  const int ns = 2;
  int rlen;
  int pna;

  memset(&prefetch, 0, sizeof(prefetch));
  memset(&cfg, 0, sizeof(cfg));
  memset(&mem, 0, sizeof(mem));
  memset(&io, 0, sizeof(io));

  offset = fdt_node_offset_by_compatible(fdt, -1,
                                         "pci-host-ecam-generic");
  if (offset < 0)
    {
      return offset;
    }

  /* Get the reg address, 64 or 32 */

  cfg.start = fdt_get_reg_base(fdt, offset, 0);
  cfg.end = cfg.start + fdt_get_reg_size(fdt, offset);

  /* Get the ranges address */

  ranges = fdt_getprop(fdt, offset, "ranges", &rlen);
  if (ranges == NULL)
    {
      return -EINVAL;
    }

  pna = fdt_get_parent_address_cells(fdt, offset);

  for (rlen /= 4; (rlen -= na + pna + ns) >= 0; ranges += na + pna + ns)
    {
      uint32_t type = fdt32_ld(ranges);

      if ((type & FDT_PCI_TYPE_MASK) == FDT_PCI_TYPE_IO)
        {
          io.start = fdt_ld_by_cells(ranges + 1, na - 1);
          io.end = io.start + fdt_ld_by_cells(ranges + na + pna, ns);
          io.offset = fdt_ld_by_cells(ranges + na, pna) - io.start;
          io.flags = PCI_RESOURCE_IO;
        }
      else if ((type & FDT_PCI_PREFETCH) == FDT_PCI_PREFETCH ||
               (type & FDT_PCI_TYPE_MASK) == FDT_PCI_TYPE_MEM64)
        {
          prefetch.start = fdt_ld_by_cells(ranges + 1, na - 1);
          prefetch.end = prefetch.start +
                         fdt_ld_by_cells(ranges + na + pna, ns);
          prefetch.offset = fdt_ld_by_cells(ranges + na, pna) -
                            prefetch.start;
          if ((type & FDT_PCI_PREFETCH) == FDT_PCI_PREFETCH)
            {
              prefetch.flags = PCI_RESOURCE_PREFETCH;
            }

          if ((type & FDT_PCI_TYPE_MASK) == FDT_PCI_TYPE_MEM64)
            {
              prefetch.flags |= PCI_RESOURCE_MEM_64;
            }
          else
            {
              prefetch.flags |= PCI_RESOURCE_MEM;
            }
        }
      else
        {
          mem.start = fdt_ld_by_cells(ranges + 1, na - 1);
          mem.end = mem.start + fdt_ld_by_cells(ranges + na + pna, ns);
          mem.offset = fdt_ld_by_cells(ranges + na, pna) - mem.start;
          mem.flags = PCI_RESOURCE_MEM;
        }
    }

  return pci_ecam_register(domain, &cfg, &io, &mem, &prefetch);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdt_pci_ecam_register/_domain
 *
 * Description:
 *   Register one or all ECAM instances
 *
 * Input Parameters:
 *   fdt      - Device tree handle
 *   domain   - The specific domain id
 *
 * Returned Value:
 *   Return 0 if success, nageative if failed
 *
 ****************************************************************************/

int fdt_pci_ecam_register(FAR const void *fdt)
{
  int offset = -1;
  int rc;

  DEBUGASSERT(fdt);

  for (; ; )
    {
      offset = fdt_node_offset_by_compatible(fdt, offset,
                                             "pci-host-ecam-generic");
      if (offset < 0)
        {
          break;
        }

      rc = fdt_pci_ecam_register_offset(fdt, offset);
      if (rc < 0)
        {
          pcierr("Failed scaning: %d\n", rc);
          return rc;
        }
    }

  return 0;
}

int fdt_pci_ecam_register_domain(FAR const void *fdt, uint16_t domain)
{
  uint32_t d;
  int offset = -1;
  int rc;

  DEBUGASSERT(fdt);

  for (; ; )
    {
      offset = fdt_node_offset_by_compatible(fdt, offset,
                                             "pci-host-ecam-generic");
      if (offset < 0)
        {
          break;
        }

      rc = fdt_load_prop_u32(fdt, offset, "linux,pci-domain", 0, &d);
      if (rc < 0)
        {
          pcierr("Failed loading domain: %d\n", rc);
          return rc;
        }

      rc = fdt_pci_ecam_register_offset(fdt, offset);
      if (rc < 0 && rc != -EEXIST)
        {
          return rc;
        }
    }

  return OK;
}
