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

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/lib/math32.h>
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

#define IS_ALIGNED(x, a) (((x) & ((a) - 1)) == 0)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pci_ecam_read_config(FAR struct pci_bus_s *bus,
                                unsigned int devfn, int where, int size,
                                FAR uint32_t *val);

static int pci_ecam_write_config(FAR struct pci_bus_s *bus,
                                 unsigned int devfn, int where, int size,
                                 uint32_t val);

static int pci_ecam_read_io(FAR struct pci_bus_s *bus, uintptr_t addr,
                            int size, FAR uint32_t *val);

static int pci_ecam_write_io(FAR struct pci_bus_s *bus, uintptr_t addr,
                             int size, uint32_t val);

static int pci_ecam_get_irq(FAR struct pci_bus_s *bus, uint32_t devfn,
                            uint8_t line, uint8_t pin);

#ifdef CONFIG_PCI_MSIX
static int pci_ecam_alloc_irq(FAR struct pci_bus_s *bus, FAR int *irq,
                              int num);

static void pci_ecam_release_irq(FAR struct pci_bus_s *bus, FAR int *irq,
                                 int num);

static int pci_ecam_connect_irq(FAR struct pci_bus_s *bus, FAR int *irq,
                                int num, FAR uintptr_t *mar,
                                FAR uint32_t *mdr);
#endif

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
  .read        = pci_ecam_read_config,
  .write       = pci_ecam_write_config,
  .read_io     = pci_ecam_read_io,
  .write_io    = pci_ecam_write_io,
  .get_irq     = pci_ecam_get_irq,
#ifdef CONFIG_PCI_MSIX
  .alloc_irq   = pci_ecam_alloc_irq,
  .release_irq = pci_ecam_release_irq,
  .connect_irq = pci_ecam_connect_irq,
#endif
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
  int num_buses = div_round_up(pci_resource_size(&ecam->cfg), 1 << 20);

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

static int pci_ecam_read_config(FAR struct pci_bus_s *bus,
                                unsigned int devfn, int where, int size,
                                FAR uint32_t *val)
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

static int pci_ecam_write_config(FAR struct pci_bus_s *bus,
                                 unsigned int devfn, int where, int size,
                                 uint32_t val)
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
 * Name: pci_ecam_read_io
 *
 * Description:
 *   Read data from the specific address.
 *
 * Input Parameters:
 *   bus  - The bus on this to read reg data
 *   addr - Data address
 *   size - Data size
 *   val  - Return value to this var
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int pci_ecam_read_io(FAR struct pci_bus_s *bus, uintptr_t addr,
                            int size, FAR uint32_t *val)
{
  if (!IS_ALIGNED(addr, size))
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
 * Name: pci_ecam_read_io
 *
 * Description:
 *   Write data to the specific address.
 *
 * Input Parameters:
 *   bus  - The bus on this to read reg data
 *   addr - Data address
 *   size - Data size
 *   val  - Write this value to specific address
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int pci_ecam_write_io(FAR struct pci_bus_s *bus, uintptr_t addr,
                             int size, uint32_t val)
{
  if (!IS_ALIGNED(addr, size))
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

#ifdef CONFIG_PCI_MSIX
static int pci_ecam_alloc_irq(FAR struct pci_bus_s *bus, FAR int *irq,
                              int num)
{
  *irq = up_alloc_irq_msi(&num);
  return num;
}

static void pci_ecam_release_irq(FAR struct pci_bus_s *bus, FAR int *irq,
                                 int num)
{
  return up_release_irq_msi(irq, num);
}

static int pci_ecam_connect_irq(FAR struct pci_bus_s *bus, FAR int *irq,
                                int num, FAR uintptr_t *mar,
                                FAR uint32_t *mdr)
{
  return up_connect_irq(irq, num, mar, mdr);
}
#endif

/****************************************************************************
 * Name: pci_ecam_get_irq
 *
 * Description:
 *  Get interrupt number associated with a given INTx line.
 *
 * Input Parameters:
 *   bus   - Bus that PCI device resides
 *   devfn - The pci device and function number
 *   line  - Activated PCI legacy interrupt line
 *   pin   - Intx pin number
 *
 * Returned Value:
 *   Return interrupt number associated with a given INTx
 *
 ****************************************************************************/

static int pci_ecam_get_irq(FAR struct pci_bus_s *bus, uint32_t devfn,
                            uint8_t line, uint8_t pin)
{
  UNUSED(bus);

  return up_get_legacy_irq(devfn, line, pin);
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
