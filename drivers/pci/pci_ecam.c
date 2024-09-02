/****************************************************************************
 * drivers/pci/pci_ecam.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci.h>
#include <nuttx/pci/pci_ecam.h>
#include <nuttx/nuttx.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define readb(a)     (*(FAR volatile uint8_t *)(a))
#define writeb(v,a)  (*(FAR volatile uint8_t *)(a) = (v))
#define readw(a)     (*(FAR volatile uint16_t *)(a))
#define writew(v,a)  (*(FAR volatile uint16_t *)(a) = (v))
#define readl(a)     (*(FAR volatile uint32_t *)(a))
#define writel(v,a)  (*(FAR volatile uint32_t *)(a) = (v))

#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
#define IS_ALIGNED(x, a)  (((x) & ((a) - 1)) == 0)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pci_ecam_read_config(FAR struct pci_bus_s *bus, uint32_t devfn,
                                int where, int size, FAR uint32_t *val);

static int pci_ecam_write_config(FAR struct pci_bus_s *bus, uint32_t devfn,
                                 int where, int size, uint32_t val);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pci_ecam_s
{
  struct pci_controller_s ctrl;
  struct pci_resource_s cfg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_ops_s g_pci_ecam_ops =
{
  .read  = pci_ecam_read_config,
  .write = pci_ecam_write_config,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_ecam_from_controller
 *
 * Description:
 *   To get the ecam type from ctrl type.
 *
 * Input Parameters:
 *   ctrl - The pci controller
 *
 * Returned Value:
 *   Return the struct pci_ecam_pcie s address
 *
 ****************************************************************************/

static inline FAR struct pci_ecam_s *
pci_ecam_from_controller(FAR struct pci_controller_s *ctrl)
{
  return container_of(ctrl, struct pci_ecam_s, ctrl);
}

/****************************************************************************
 * Name: pci_ecam_conf_address
 *
 * Description:
 *   This function is used to get the specify reg address of a function
 *   configuration space.
 *
 * Input Parameters:
 *   bus   - PCI bus type private data
 *   devfn - Specify a BDF
 *   where - Which ID in configuration space
 *
 ****************************************************************************/

static FAR void *pci_ecam_conf_address(FAR const struct pci_bus_s *bus,
                                       uint32_t devfn, int where)
{
  FAR struct pci_ecam_s *ecam = pci_ecam_from_controller(bus->ctrl);
  FAR void *addr;

  addr = (FAR void *)ecam->cfg.start;
  addr += bus->number << 20;
  addr += PCI_SLOT(devfn) << 15;
  addr += PCI_FUNC(devfn) << 12;
  addr += where;

  return addr;
}

/****************************************************************************
 * Name: pci_ecam_addr_valid
 *
 * Description:
 *   To check the bus number whether or not valid.
 *
 * Input Parameters:
 *   bus   - The bus private data
 *   devfn - Get a specify dev by devfn
 *
 * Returned Value:
 *   True if success, false if failed
 *
 ****************************************************************************/

static bool pci_ecam_addr_valid(FAR const struct pci_bus_s *bus,
                                uint32_t devfn)
{
  FAR struct pci_ecam_s *ecam = pci_ecam_from_controller(bus->ctrl);
  int num_buses = DIV_ROUND_UP(pci_resource_size(&ecam->cfg), 1 << 20);

  return bus->number < num_buses;
}

/****************************************************************************
 * Name: pci_ecam_read_config
 *
 * Description:
 *   Read data from the speicfy register.
 *
 * Input Parameters:
 *   bus   - The bus on this to read reg data
 *   devfn - BDF
 *   where - Which cfg space ID
 *   size  - Data size
 *   val   - Return value to this var
 *
 * Returned Value:
 *   Return the specify enum result of operation
 *
 ****************************************************************************/

static int pci_ecam_read_config(FAR struct pci_bus_s *bus, uint32_t devfn,
                                int where, int size, FAR uint32_t *val)
{
  FAR void *addr;

  if (!pci_ecam_addr_valid(bus, devfn))
    {
      return -ENODEV;
    }

  addr = pci_ecam_conf_address(bus, devfn, where);

  if (!IS_ALIGNED((uintptr_t)addr, size))
    {
      *val = 0;
      return -EINVAL;
    }

  if (size == 4)
    {
      *val = readl(addr);
    }
  else if (size == 2)
    {
      *val = readw(addr);
    }
  else if (size == 1)
    {
      *val = readb(addr);
    }
  else
    {
      *val = 0;
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: pci_ecam_write_config
 *
 * Description:
 *   Write data into speicfy register.
 *
 * Input Parameters:
 *   bus   - The specify bus private data
 *   devfn - BDF
 *   where - Which ID in configuration space
 *   size  - Data Size
 *   val   - The value to be written
 *
 * Returned Value:
 *   Return the specify enum result of operation
 *
 ****************************************************************************/

static int pci_ecam_write_config(FAR struct pci_bus_s *bus, uint32_t devfn,
                                 int where, int size, uint32_t val)
{
  FAR void *addr;

  if (!pci_ecam_addr_valid(bus, devfn))
    {
      return -ENODEV;
    }

  addr = pci_ecam_conf_address(bus, devfn, where);

  if (!IS_ALIGNED((uintptr_t)addr, size))
    {
      return -EINVAL;
    }

  if (size == 4)
    {
      writel(val, addr);
    }
  else if (size == 2)
    {
      writew(val, addr);
    }
  else if (size == 1)
    {
      writeb(val, addr);
    }
  else
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_ecam_register
 *
 * Description:
 *   This function is used to register an ecam driver for pci.
 *
 * Input Parameters:
 *   cfg      - Configuration space data
 *   io       - I/O space data
 *   mem      - No-prefetchable space data
 *   mem_pref - Prefetchable space data
 *
 * Returned Value:
 *   Return 0 if success, nageative if failed
 *
 ****************************************************************************/

int pci_ecam_register(FAR const struct pci_resource_s *cfg,
                      FAR const struct pci_resource_s *io,
                      FAR const struct pci_resource_s *mem,
                      FAR const struct pci_resource_s *mem_pref)
{
  FAR struct pci_ecam_s *ecam;

  if (cfg == NULL || (io == NULL && mem == NULL && mem_pref == NULL))
    {
      return -EINVAL;
    }

  ecam = kmm_zalloc(sizeof(*ecam));
  if (ecam == NULL)
    {
      return -ENOMEM;
    }

  memcpy(&ecam->cfg, cfg, sizeof(*cfg));

  if (io != NULL)
    {
      memcpy(&ecam->ctrl.io, io, sizeof(*io));
    }

  if (mem != NULL)
    {
      memcpy(&ecam->ctrl.mem, mem, sizeof(*mem));
    }

  if (mem_pref != NULL)
    {
      memcpy(&ecam->ctrl.mem_pref, mem_pref, sizeof(*mem_pref));
    }

  ecam->ctrl.ops = &g_pci_ecam_ops;
  return pci_register_controller(&ecam->ctrl);
}
