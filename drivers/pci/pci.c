/****************************************************************************
 * drivers/pci/pci.c
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
#include <sys/pciio.h>
#include <sys/endian.h>

#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALIGN(x, m) (((x) + ((m) - 1)) & ~((uintptr_t)(m) - 1))

/*  Wrappers for all PCI configuration access functions.  They just check
 *  alignment, do locking and call the low-level functions pointed to
 *  by ctrl->ops.
 */

#define PCI_byte_BAD  0
#define PCI_word_BAD  ((where) & 1)
#define PCI_dword_BAD ((where) & 3)

#define PCI_BUS_READ_CONFIG(len, type, size)                                   \
  int pci_bus_read_config_##len(FAR struct pci_bus_s *bus, unsigned int devfn, \
                                int where, FAR type *value)                    \
  {                                                                            \
    int ret = -EINVAL;                                                         \
    uint32_t data = 0;                                                         \
                                                                               \
    if (!PCI_##len##_BAD)                                                      \
      {                                                                        \
        ret = pci_bus_read_config(bus, devfn, where, size, &data);             \
      }                                                                        \
                                                                               \
    *value = (type)data;                                                       \
    return ret;                                                                \
  }

#define PCI_BUS_WRITE_CONFIG(len, type, size)                                   \
  int pci_bus_write_config_##len(FAR struct pci_bus_s *bus, unsigned int devfn, \
                                 int where, type value)                         \
  {                                                                             \
    int ret = -EINVAL;                                                          \
                                                                                \
    if (!PCI_##len##_BAD)                                                       \
      {                                                                         \
        ret = pci_bus_write_config(bus, devfn, where, size, value);             \
      }                                                                         \
                                                                                \
    return ret;                                                                 \
  }

#define PCI_BUS_READ_IO(len, type, size)                                        \
  int pci_bus_read_io_##len(FAR struct pci_bus_s *bus, uintptr_t where,         \
                            FAR type *value)                                    \
  {                                                                             \
    int ret = -EINVAL;                                                          \
    uint32_t data = 0;                                                          \
                                                                                \
    if (!PCI_##len##_BAD)                                                       \
      {                                                                         \
        ret = pci_bus_read_io(bus, where, size, &data);                         \
      }                                                                         \
                                                                                \
    *value = (type)data;                                                        \
    return ret;                                                                 \
  }

#define PCI_BUS_WRITE_IO(len, type, size)                                       \
  int pci_bus_write_io_##len(FAR struct pci_bus_s *bus, uintptr_t where,        \
                             type value)                                        \
  {                                                                             \
    int ret = -EINVAL;                                                          \
                                                                                \
    if (!PCI_##len##_BAD)                                                       \
      {                                                                         \
        ret = pci_bus_write_io(bus, where, size, value);                        \
      }                                                                         \
                                                                                \
    return ret;                                                                 \
  }

#define pci_match_one_device(id, dev)                               \
  (((id)->vendor == PCI_ANY_ID || (id)->vendor == (dev)->vendor) && \
   ((id)->device == PCI_ANY_ID || (id)->device == (dev)->device) && \
   ((id)->subvendor == PCI_ANY_ID ||                                \
    (id)->subvendor == (dev)->subsystem_vendor) &&                  \
   ((id)->subdevice == PCI_ANY_ID ||                                \
    (id)->subdevice == (dev)->subsystem_device) &&                  \
   ((((id)->class ^ (dev)->class) & (id)->class_mask) == 0))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pci_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_pci_lock = NXMUTEX_INITIALIZER;
static struct list_node g_pci_device_list =
                        LIST_INITIAL_VALUE(g_pci_device_list);
static struct list_node g_pci_driver_list =
                        LIST_INITIAL_VALUE(g_pci_driver_list);
static struct list_node g_pci_ctrl_list =
                        LIST_INITIAL_VALUE(g_pci_ctrl_list);

static const struct file_operations g_pci_fops =
{
  NULL,                  /* open */
  NULL,                  /* close */
  NULL,                  /* read */
  NULL,                  /* write */
  NULL,                  /* seek */
  pci_ioctl,             /* ioctl */
  NULL,                  /* mmap */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct pci_device_s *
pci_do_find_device_from_bus(FAR struct pci_bus_s *bus, uint8_t busno,
                         unsigned int devfn)
{
  FAR struct pci_bus_s *bus_tmp;
  FAR struct pci_device_s *dev;

  list_for_every_entry(&bus->devices, dev, struct pci_device_s, node)
    {
      if (dev->bus->number == busno && devfn == dev->devfn)
        {
          return dev;
        }
    }

  list_for_every_entry(&bus->children, bus_tmp,
                       struct pci_bus_s, node)
    {
      dev = pci_find_device_from_bus(bus_tmp, busno, devfn);
      if (dev != NULL)
        {
          return dev;
        }
    }

  return NULL;
}

static int pci_vpd_read(FAR struct pci_bus_s *bus, uint32_t devfn,
                        int offset, int count, FAR uint32_t *data)
{
  uint8_t pos;
  int i;

  if (offset + count >= PCI_VPD_ADDR_MASK || data != NULL)
    {
      return -EINVAL;
    }

  pos = pci_bus_find_capability(bus, devfn, PCI_CAP_ID_VPD);
  if (pos == 0)
    {
      return -ENOENT;
    }

  for (i = 0; i < count; i++, offset += 4)
    {
      int j = 0;

      pci_bus_write_config_word(bus, devfn, pos + PCI_VPD_ADDR, offset);

      /**
       * PCI 2.2 does not specify how long we should poll
       * for completion nor whether the operation can fail.
       */

      for (; ; )
        {
          uint16_t addr;

          pci_bus_read_config_word(bus, devfn, pos + PCI_VPD_ADDR, &addr);
          if (addr & PCI_VPD_ADDR_F)
            {
              break;
            }

          if (++j == 20)
            {
              return -EIO;
            }

          up_udelay(4);
        }

      pci_bus_read_config_dword(bus, devfn, pos + PCI_VPD_DATA, &data[i]);
      data[i] = le32toh(data[i]);
    }

  return 0;
}

/****************************************************************************
 * Name: pci_ioctl
 *
 * Description:
 *  for lspci read/write pci config space
 *
 * Input Parameters:
 *   filep - The open file description
 *   cmd - The cmd to read/write cmd
 *   arg - The arg to pass ioctl
 * Returned Value:
 *   The length of the param
 *
 ****************************************************************************/

static int pci_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct pci_controller_s *ctrl;
  FAR struct pcisel *sel;
  uint32_t devfn;
  uint8_t i = 0;
  int ret;

  sel = (FAR struct pcisel *)arg;
  devfn = PCI_DEVFN(sel->pc_dev, sel->pc_func);

  ret = nxmutex_lock(&g_pci_lock);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry(&g_pci_ctrl_list, ctrl, struct pci_controller_s, node)
    {
      if (i == sel->pc_domain)
        {
          break;
        }

      i++;
    }

  nxmutex_unlock(&g_pci_lock);

  if (i != sel->pc_domain)
    {
      return -ENODEV;
    }

  switch (cmd)
    {
      case PCIOCREAD:
        {
          FAR struct pci_io *io = (FAR struct pci_io *)arg;
          ret = pci_bus_read_config(ctrl->bus, devfn, io->pi_reg,
                                    io->pi_width, &io->pi_data);
          break;
         }

      case PCIOCWRITE:
        {
          FAR struct pci_io *io = (FAR struct pci_io *)arg;
          ret = pci_bus_write_config(ctrl->bus, devfn, io->pi_reg,
                                     io->pi_width, io->pi_data);
          break;
        }

      case PCIOCGETROMLEN:
        {
          FAR struct pci_rom *rom = (FAR struct pci_rom *)arg;
          FAR struct pci_device_s *dev =
            pci_find_device_from_bus(ctrl->bus, sel->pc_bus, devfn);
          if (dev == NULL)
            {
              return -ENODEV;
            }

          rom->pr_romlen = pci_resource_len(dev, PCI_ROM_RESOURCE);
          ret = 0;
          break;
        }

      case PCIOCGETROM:
        {
          FAR void *p;
          uint32_t addr;
          uint32_t len;

          FAR struct pci_rom *rom = (FAR struct pci_rom *)arg;
          FAR struct pci_device_s *dev =
            pci_find_device_from_bus(ctrl->bus, sel->pc_bus, devfn);
          if (dev == NULL)
            {
              return -ENODEV;
            }

          addr = pci_resource_start(dev, PCI_ROM_RESOURCE);
          len = pci_resource_len(dev, PCI_ROM_RESOURCE);
          if (rom->pr_romlen < len)
            {
              rom->pr_romlen = len;
              ret = -E2BIG;
              break;
            }

          p = pci_map_bar(dev, PCI_ROM_RESOURCE);
          if (p == NULL)
            {
              ret = -ENOENT;
              break;
            }

          pci_bus_write_config_dword(ctrl->bus, devfn, PCI_ROM_ADDRESS,
                                     addr | PCI_ROM_ADDRESS_ENABLE);
          memcpy(rom->pr_rom, p, len);
          pci_bus_write_config_dword(ctrl->bus, devfn,
                                     PCI_ROM_ADDRESS, addr);
          break;
        }

      case PCIOCREADMASK:
        {
          uint32_t data;

          FAR struct pci_io *io = (FAR struct pci_io *)arg;
          if (io->pi_width != 4 || (io->pi_reg & 0x3) ||
              io->pi_reg <  PCI_BASE_ADDRESS_0 ||
              io->pi_reg >= PCI_BASE_ADDRESS_SPACE)
            {
              ret = -EINVAL;
              break;
            }

          pci_bus_read_config_dword(ctrl->bus, devfn, io->pi_reg, &data);
          pci_bus_write_config_dword(ctrl->bus, devfn, io->pi_reg,
                                     0xffffffff);
          pci_bus_read_config_dword(ctrl->bus, devfn,
                                    io->pi_reg, &io->pi_data);
          pci_bus_write_config_dword(ctrl->bus, devfn, io->pi_reg, data);
          break;
        }

      case PCIOCGETVPD:
        {
          FAR struct pci_vpd_req *pv = (FAR struct pci_vpd_req *)arg;
          ret = pci_vpd_read(ctrl->bus, devfn, pv->pv_offset,
                             pv->pv_count, pv->pv_data);
          break;
        }

      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: pci_change_master
 *
 * Description:
 *   Enables/Disbale bus-mastering for device dev
 *
 * Input Parameters:
 *   dev    - The PCI device to cchange
 *   enable - True to enable, False to disable
 *
 ****************************************************************************/

static void pci_change_master(FAR struct pci_device_s *dev, bool enable)
{
  uint16_t old_cmd;
  uint16_t cmd;

  pci_read_config_word(dev, PCI_COMMAND, &old_cmd);
  if (enable)
    {
      cmd = old_cmd | PCI_COMMAND_MASTER;
    }
  else
    {
      cmd = old_cmd & ~PCI_COMMAND_MASTER;
    }

  if (cmd != old_cmd)
    {
      pci_write_config_word(dev, PCI_COMMAND, cmd);
    }
}

/****************************************************************************
 * Name: pci_bus_find_start_cap
 *
 * Description:
 *   Find the offset of first capability list entry.
 *
 * Input Parameters:
 *   bus      - The bus this dev exist on
 *   devfn    - BDF
 *   hdr_type - Cfg space head type ID
 *
 * Returned Value:
 *   Return the ID of first capability list entry
 *
 ****************************************************************************/

static uint8_t pci_bus_find_start_cap(FAR struct pci_bus_s *bus,
                                      unsigned int devfn,
                                      uint8_t hdr_type)
{
  uint16_t status;

  pci_bus_read_config_word(bus, devfn, PCI_STATUS, &status);
  if (!(status & PCI_STATUS_CAP_LIST))
    {
      return 0;
    }

  /* Ignore MF bit */

  switch (hdr_type & 0x7f)
  {
    case PCI_HEADER_TYPE_NORMAL:
    case PCI_HEADER_TYPE_BRIDGE:
      return PCI_CAPABILITY_LIST;

    case PCI_HEADER_TYPE_CARDBUS:
      return PCI_CB_CAPABILITY_LIST;

    default:
      return 0;
  }
}

/****************************************************************************
 * Name: pci_find_next_cap_ttl
 *
 * Description:
 *
 * Input Parameters:
 *   dev   - The PCI device to find capbilities
 *   devfn - BDF
 *   pos   - List node
 *   cap   - Value of capabilities
 *   ttl   - The max depth to find
 *
 ****************************************************************************/

static uint8_t pci_find_next_cap_ttl(FAR struct pci_bus_s *bus,
                                     unsigned int devfn, uint8_t pos,
                                     int cap, FAR int *ttl)
{
  uint16_t ent;
  uint8_t id;

  pci_bus_read_config_byte(bus, devfn, pos, &pos);

  while ((*ttl)--)
    {
      if (pos < 0x40)
        {
          break;
        }

      pos &= ~3;
      pci_bus_read_config_word(bus, devfn, pos, &ent);

      id = ent & 0xff;
      if (id == 0xff)
        {
          break;
        }

      if (id == cap)
        {
          return pos;
        }

      pos = ent >> 8;
    }

  return 0;
}

/****************************************************************************
 * Name: pci_find_next_cap
 *
 * Description:
 *   To find the next capability.
 *
 * Input Parameters:
 *   dev   - The PCI device to find capbilities
 *   devfn - BDF
 *   pos   - List node
 *   cap   - Value of capabilities
 *
 * Returned Value:
 *   Return the capability data
 *
 ****************************************************************************/

static uint8_t pci_find_next_cap(FAR struct pci_bus_s *bus,
                                 unsigned int devfn, uint8_t pos, int cap)
{
  int ttl = 48;
  return pci_find_next_cap_ttl(bus, devfn, pos, cap, &ttl);
}

/****************************************************************************
 * Name: pci_alloc_bus
 *
 * Description:
 *   Alloc a memory for a bus and init list node.
 *
 * Returned Value:
 *   Return the memory address alloced
 *
 ****************************************************************************/

static FAR struct pci_bus_s *pci_alloc_bus(void)
{
  FAR struct pci_bus_s *bus;

  bus = kmm_zalloc(sizeof(*bus));
  if (bus == NULL)
    {
      return NULL;
    }

  list_initialize(&bus->node);
  list_initialize(&bus->children);
  list_initialize(&bus->devices);

  return bus;
}

/****************************************************************************
 * Name: pci_alloc_device
 *
 * Description:
 *   Alloc a memory for device be scanned and init the list node.
 *
 * Returned Value:
 *   Return the device address alloced
 *
 ****************************************************************************/

static FAR struct pci_device_s *pci_alloc_device(void)
{
  FAR struct pci_device_s *dev;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return NULL;
    }

  list_initialize(&dev->node);
  list_initialize(&dev->bus_list);

  return dev;
}

/****************************************************************************
 * Name: pci_register_bus_devices
 *
 * Description:
 *   Register all devices scanned and all buses scanned to responsing list.
 *
 * Input Parameters:
 *   bus - The boot bus
 *
 ****************************************************************************/

static void pci_register_bus_devices(FAR struct pci_bus_s *bus)
{
  FAR struct pci_device_s *dev;
  FAR struct pci_bus_s *child_bus;

  /* Activate all devices on this bus */

  list_for_every_entry(&bus->devices, dev, struct pci_device_s, bus_list)
    {
      pci_register_device(dev);
    }

  /* Walk down the hierarchy */

  list_for_every_entry(&bus->children, child_bus, struct pci_bus_s, node)
    {
      pci_register_bus_devices(child_bus);
    }
}

/****************************************************************************
 * Name: pci_size
 *
 * Description:
 *   To calculate the pci dev address size.
 *
 * Input Parameters:
 *   base    - PCI address base address
 *   maxbase - PCI max base address
 *   mask    - PCI addres mask
 *
 * Returned Value:
 *   Return the size result
 *
 ****************************************************************************/

static uint32_t pci_size(uint32_t base, uint32_t maxbase, uint32_t mask)
{
  uint32_t size = maxbase & mask;

  if (size == 0)
    {
      return 0;
    }

  size = (size & ~(size - 1)) - 1;

  if (base == maxbase && ((base | size) & mask) != mask)
    {
      return 0;
    }

  return size + 1;
}

/****************************************************************************
 * Name: pci_setup_device
 *
 * Description:
 *   Search every bar in the device be found, mapping memory if MEM or
 *   prefetchable MEM, and add this dev to the device list.
 *
 * Input Parameters:
 *   dev     - The PCI device be found
 *   max_bar - Max bar number(6 or 2)
 *   rom_addr - The pci device rom addr
 *
 ****************************************************************************/

static void pci_setup_device(FAR struct pci_device_s *dev, int max_bar,
                             uint8_t rom_addr)
{
  int bar;
  uint32_t orig;
  uint32_t mask;
  uint32_t size;
  uintptr_t start;
#ifdef CONFIG_PCI_ASSIGN_ALL_BUSES
  uint8_t cmd;

  pci_read_config_byte(dev, PCI_COMMAND, &cmd);
  pci_write_config_byte(dev, PCI_COMMAND,
                        cmd & ~PCI_COMMAND_IO & ~PCI_COMMAND_MEMORY);
#else
  uint32_t tmp;
#endif

  for (bar = 0; bar < max_bar; bar++)
    {
      int base_address_0 = PCI_BASE_ADDRESS_0 + bar * 4;
      int base_address_1 = PCI_BASE_ADDRESS_1 + bar * 4;
      FAR struct pci_resource_s *res;
      unsigned int flags;

      pci_read_config_dword(dev, base_address_0, &orig);
      pci_write_config_dword(dev, base_address_0, 0xfffffffe);
      pci_read_config_dword(dev, base_address_0, &mask);
      pci_write_config_dword(dev, base_address_0, orig);

      if (mask == 0 || mask == 0xffffffff)
        {
          pciinfo("pbar%d set bad mask\n", bar);
          continue;
        }

      if (mask & PCI_BASE_ADDRESS_SPACE_IO)
        {
          /* IO */

          size  = pci_size(orig, mask, 0xfffffffe);
          flags = PCI_RESOURCE_IO;
          res   = &dev->bus->ctrl->io;
        }
      else if ((mask & PCI_BASE_ADDRESS_MEM_PREFETCH) &&
               pci_resource_size(&dev->bus->ctrl->mem_pref))
        {
          /* Prefetchable MEM */

          size  = pci_size(orig, mask, 0xfffffff0);
          flags = PCI_RESOURCE_MEM | PCI_RESOURCE_PREFETCH;
          res   = &dev->bus->ctrl->mem_pref;
        }
      else
        {
          /* Non-prefetch MEM */

          size  = pci_size(orig, mask, 0xfffffff0);
          flags = PCI_RESOURCE_MEM;
          res   = &dev->bus->ctrl->mem;
        }

      if (size == 0)
        {
          pcierr("pbar%d bad mask\n", bar);
          continue;
        }

      pciinfo("pbar%d: mask=%08" PRIx32 " %" PRIu32 "bytes\n",
              bar, mask, size);

#ifdef CONFIG_PCI_ASSIGN_ALL_BUSES
      if (ALIGN(res->start, size) + size > res->end)
        {
          pcierr("pbar%d: does not fit within bus res\n", bar);
          return;
        }

      res->start = ALIGN(res->start, size);
      pci_write_config_dword(dev, base_address_0, res->start);
      if (mask & PCI_BASE_ADDRESS_MEM_TYPE_64)
        {
          pci_write_config_dword(dev, base_address_1, res->start >> 32);
        }

      start = res->start;
      res->start += size;
#else
      UNUSED(res);
      pci_read_config_dword(dev, base_address_0, &tmp);
      if (mask & PCI_BASE_ADDRESS_SPACE_IO)
        {
          start = tmp & PCI_BASE_ADDRESS_IO_MASK;
        }
      else
        {
          start = tmp & PCI_BASE_ADDRESS_MEM_MASK;
        }

      if (mask & PCI_BASE_ADDRESS_MEM_TYPE_64)
        {
          pci_read_config_dword(dev, base_address_1, &tmp);
          start |= (uint64_t)tmp << 32;
        }
#endif

      dev->resource[bar].flags = flags;
      dev->resource[bar].start = start;
      dev->resource[bar].end   = start + size - 1;

      if (mask & PCI_BASE_ADDRESS_MEM_TYPE_64)
        {
          dev->resource[bar++].flags |= PCI_RESOURCE_MEM_64;
        }
    }

  pci_read_config_dword(dev, rom_addr, &orig);
  pci_write_config_dword(dev, rom_addr,
                         ~PCI_ROM_ADDRESS_ENABLE);
  pci_read_config_dword(dev, rom_addr, &mask);
  pci_write_config_dword(dev, rom_addr, orig);
  start = PCI_ROM_ADDR(orig);
  size = PCI_ROM_SIZE(mask);
  if (start != 0 && size != 0)
    {
      dev->resource[PCI_ROM_RESOURCE].flags |=
        PCI_RESOURCE_MEM | PCI_RESOURCE_PREFETCH;
      dev->resource[PCI_ROM_RESOURCE].start = start;
      dev->resource[PCI_ROM_RESOURCE].end = start + size - 1;
    }

#ifdef CONFIG_PCI_ASSIGN_ALL_BUSES
  pci_write_config_byte(dev, PCI_COMMAND, cmd);
#endif

  list_add_tail(&dev->bus->devices, &dev->bus_list);
}

/****************************************************************************
 * Name: pci_presetup_bridge
 *
 * Description:
 *    Setup data to the next bridge register.
 *
 * Input Parameters:
 *    dev - The next bridge dev
 *
 ****************************************************************************/

static void pci_presetup_bridge(FAR struct pci_device_s *dev)
{
#ifndef CONFIG_PCI_ASSIGN_ALL_BUSES
  pci_read_config_byte(dev, PCI_PRIMARY_BUS, &dev->bus->number);
  pci_read_config_byte(dev, PCI_SECONDARY_BUS, &dev->subordinate->number);
#else
  FAR struct pci_controller_s *ctrl = dev->bus->ctrl;
  uint16_t cmdstat;

  pci_read_config_word(dev, PCI_COMMAND, &cmdstat);

  /* Configure bus number registers */

  pci_write_config_byte(dev, PCI_PRIMARY_BUS, dev->bus->number);
  pci_write_config_byte(dev, PCI_SECONDARY_BUS, dev->subordinate->number);
  pci_write_config_byte(dev, PCI_SUBORDINATE_BUS, 0xff);

  if (pci_resource_size(&ctrl->mem))
    {
      /* Set up memory and I/O filter limits, assume 32-bit I/O space */

      ctrl->mem.start = ALIGN(ctrl->mem.start, 1024 * 1024);
      pci_write_config_word(dev, PCI_MEMORY_BASE,
                            (ctrl->mem.start & 0xfff00000) >> 16);
      cmdstat |= PCI_COMMAND_MEMORY;
    }
  else
    {
      pci_write_config_word(dev, PCI_MEMORY_BASE, 0x1000);
      pci_write_config_word(dev, PCI_MEMORY_LIMIT, 0x0);
    }

  if (pci_resource_size(&ctrl->mem_pref))
    {
      /* Set up memory and I/O filter limits, assume 32-bit I/O space */

      ctrl->mem_pref.start = ALIGN(ctrl->mem_pref.start, 1024 * 1024);
      pci_write_config_word(dev, PCI_PREF_MEMORY_BASE,
                            (ctrl->mem_pref.start & 0xfff00000) >> 16);
      pci_write_config_dword(dev, PCI_PREF_BASE_UPPER32,
                             ctrl->mem_pref.start >> 32);
      cmdstat |= PCI_COMMAND_MEMORY;
    }
  else
    {
      /* We don't support prefetchable memory for now, so disable */

      pci_write_config_word(dev, PCI_PREF_MEMORY_BASE, 0x1000);
      pci_write_config_word(dev, PCI_PREF_MEMORY_LIMIT, 0x0);
      pci_write_config_dword(dev, PCI_PREF_BASE_UPPER32, 0x0);
      pci_write_config_dword(dev, PCI_PREF_LIMIT_UPPER32, 0x0);
    }

  if (pci_resource_size(&ctrl->io))
    {
      ctrl->io.start = ALIGN(ctrl->io.start, 1024 * 4);
      pci_write_config_byte(dev, PCI_IO_BASE,
                            (ctrl->io.start & 0x0000f000) >> 8);
      pci_write_config_word(dev, PCI_IO_BASE_UPPER16,
                            (ctrl->io.start & 0xffff0000) >> 16);
      cmdstat |= PCI_COMMAND_IO;
    }

  /* Enable memory and I/O accesses, enable bus master */

  pci_write_config_word(dev, PCI_COMMAND, cmdstat | PCI_COMMAND_MASTER);
#endif
}

/****************************************************************************
 * Name: pci_postsetup_bridge
 *
 * Description:
 *   Setup data into Limit subordinate reg.
 *
 * Input Parameters:
 *   dev - The next bridge dev
 *
 ****************************************************************************/

static void pci_postsetup_bridge(FAR struct pci_device_s *dev)
{
#ifdef CONFIG_PCI_ASSIGN_ALL_BUSES
  FAR struct pci_controller_s *ctrl = dev->bus->ctrl;

  /* Limit subordinate to last used bus number */

  pci_write_config_byte(dev, PCI_SUBORDINATE_BUS, ctrl->busno - 1);

  if (pci_resource_size(&ctrl->mem))
    {
      ctrl->mem.start = ALIGN(ctrl->mem.start, 1024 * 1024);
      pciinfo("bridge NP limit at %" PRIxPTR "\n", ctrl->mem.start);
      pci_write_config_word(dev, PCI_MEMORY_LIMIT,
                            ((ctrl->mem.start - 1) & 0xfff00000) >> 16);
    }

  if (pci_resource_size(&ctrl->mem_pref))
    {
      ctrl->mem_pref.start = ALIGN(ctrl->mem_pref.start, 1024 * 1024);
      pciinfo("bridge P limit at %" PRIxPTR "\n", ctrl->mem_pref.start);
      pci_write_config_word(dev, PCI_PREF_MEMORY_LIMIT,
                            ((ctrl->mem_pref.start - 1) & 0xfff00000) >> 16);
      pci_write_config_dword(dev, PCI_PREF_LIMIT_UPPER32,
                             (ctrl->mem_pref.start - 1) >> 32);
    }

  if (pci_resource_size(&ctrl->io))
    {
      ctrl->io.start = ALIGN(ctrl->io.start, 1024 * 4);
      pciinfo("bridge IO limit at %" PRIxPTR "\n", ctrl->io.start);
      pci_write_config_byte(dev, PCI_IO_LIMIT,
                            ((ctrl->io.start - 1) & 0x0000f000) >> 8);
      pci_write_config_word(dev, PCI_IO_LIMIT_UPPER16,
                            ((ctrl->io.start - 1) & 0xffff0000) >> 16);
    }
#endif
}

/****************************************************************************
 * Name: pci_scan_bus
 *
 * Description:
 *  Iterates over all slots on bus looking for devices and buses to
 *  enumerate.
 *
 * Input Parameters:
 *   bus - The root bus device that lets us address the whole tree
 *
 ****************************************************************************/

static void pci_scan_bus(FAR struct pci_bus_s *bus)
{
  FAR struct pci_device_s *dev;
  FAR struct pci_bus_s *child_bus;
  unsigned int devfn;
  uint32_t l;
  uint32_t class;
  uint8_t hdr_type;
  uint8_t is_multi = 0;

  pciinfo("pci_scan_bus for bus %d\n", bus->number);

  for (devfn = 0; devfn < 0xff; ++devfn)
    {
      if (PCI_FUNC(devfn) && !is_multi)
        {
          /* Not a multi-function device */

          continue;
        }

      if (pci_bus_read_config_byte(bus, devfn, PCI_HEADER_TYPE, &hdr_type))
        {
          continue;
        }

      if (!PCI_FUNC(devfn))
        {
          is_multi = hdr_type & 0x80;
        }

      /* Some broken boards return 0 if a slot is empty: */

      if (pci_bus_read_config_dword(bus, devfn, PCI_VENDOR_ID, &l) ||
          l == 0xffffffff || l == 0x00000000 || l == 0x0000ffff ||
          l == 0xffff0000)
        {
          continue;
        }

      dev = pci_alloc_device();
      dev->bus = bus;
      dev->devfn = devfn;
      dev->vendor = l & 0xffff;
      dev->device = (l >> 16) & 0xffff;
      dev->hdr_type = hdr_type;

      pci_read_config_dword(dev, PCI_CLASS_REVISION, &class);
      dev->revision = class & 0xff;

      /* Upper 3 bytes */

      class >>= 8;
      dev->class = class;
      class >>= 8;

      pciinfo("class = %08" PRIx32 ", hdr_type = %08x\n", class, hdr_type);
      pciinfo("%02x:%02" PRIx32 " [%04x:%04x]\n", bus->number, dev->devfn,
              dev->vendor, dev->device);

      switch (hdr_type & 0x7f)
      {
        case PCI_HEADER_TYPE_NORMAL:
          if (class == PCI_CLASS_BRIDGE_PCI)
            {
              goto bad;
            }

          pci_setup_device(dev, 6, PCI_ROM_ADDRESS);

          pci_read_config_word(dev, PCI_SUBSYSTEM_ID,
                               &dev->subsystem_device);
          pci_read_config_word(dev, PCI_SUBSYSTEM_VENDOR_ID,
                               &dev->subsystem_vendor);
          break;

        case PCI_HEADER_TYPE_BRIDGE:
          child_bus = pci_alloc_bus();

          /* Inherit parent properties */

          child_bus->ctrl = bus->ctrl;
          child_bus->parent_bus = bus;

#ifdef CONFIG_PCI_ASSIGN_ALL_BUSES
          child_bus->number = bus->ctrl->busno++;
#endif

          list_add_tail(&bus->children, &child_bus->node);
          dev->subordinate = child_bus;

          /* Scan pci hierarchy behind bridge */

          pci_presetup_bridge(dev);
          pci_scan_bus(child_bus);
          pci_postsetup_bridge(dev);

          pci_setup_device(dev, 2, PCI_ROM_ADDRESS1);
          break;

        default:
        bad:
          pcierr("PCI: %02x:%02" PRIx32 " [%04x/%04x/%06" PRIx32
                 "] has unknown header type %02x, ignoring.\n",
                 bus->number, dev->devfn, dev->vendor,
                 dev->device, class, hdr_type);
          continue;
      }
    }
}

/****************************************************************************
 * Name: pci_get_msi_base
 *
 * Description:
 *  Get MSI and MSI-X base
 *
 * Input Parameters:
 *   dev  - device
 *   msi  - returned MSI base
 *   msix - returned MSI-X base
 *
 * Return value:
 *   None
 *
 ****************************************************************************/

static void pci_get_msi_base(FAR struct pci_device_s *dev, FAR uint8_t *msi,
                             FAR uint8_t *msix)
{
  if (msi != NULL)
    {
      *msi = pci_find_capability(dev, PCI_CAP_ID_MSI);
    }

  if (msix != NULL)
    {
      *msix = pci_find_capability(dev, PCI_CAP_ID_MSIX);
    }
}

/****************************************************************************
 * Name: pci_enable_msi
 *
 * Description:
 *   Configure and enable MSI.
 *
 * Input Parameters:
 *   dev - device
 *   irq - allocated vectors
 *   num - number of vectors
 *   msi - MSI base address
 *
 * Return value:
 *   OK on success or a negative error code on failure
 *
 ****************************************************************************/

static int pci_enable_msi(FAR struct pci_device_s *dev, FAR int *irq,
                          int num, uint8_t msi)
{
  uint32_t  mdr   = 0;
  uint16_t  flags = 0;
  uintptr_t mar   = 0;
  uint16_t  mme   = 0;
  uint32_t  mmc   = 0;
  int       ret   = OK;

  /* Suppoted messages */

  for (mme = 0; (1 << mme) < num; mme++);

  /* Get Message Control Register */

  pci_read_config_word(dev, msi + PCI_MSI_FLAGS, &flags);
  mmc = (flags & PCI_MSI_FLAGS_QMASK) >> PCI_MSI_FLAGS_QMASK_SHIFT;
  if (mme > mmc)
    {
      mme = mmc;
      num = 1 << mme;
      pciinfo("Limit MME to %"PRIx32", num to %d\n", mmc, num);
    }

  /* Configure MSI (arch-specific) */

  ret = dev->bus->ctrl->ops->connect_irq(dev->bus, irq, num, &mar, &mdr);
  if (ret < 0)
    {
      return ret;
    }

  /* Write Message Address Regsiter */

  pci_write_config_dword(dev, msi + PCI_MSI_ADDRESS_LO, mar);

  /* Write Message Data Register */

  if ((flags & PCI_MSI_FLAGS_64BIT) != 0)
    {
      pci_write_config_dword(dev, msi + PCI_MSI_ADDRESS_HI,
                             ((uint64_t)mar >> 32));
      pci_write_config_dword(dev, msi + PCI_MSI_DATA_64, mdr);
    }
  else
    {
      pci_write_config_word(dev, msi + PCI_MSI_DATA_32, mdr);
    }

  flags |= mme << PCI_MSI_FLAGS_QSIZE_SHIFT;

  /* Enable MSI */

  flags |= PCI_MSI_FLAGS_ENABLE;

  /* Write Message Control Register */

  pci_write_config_word(dev, msi + PCI_MSI_FLAGS, flags);
  return OK;
}

#ifdef CONFIG_PCI_MSIX
/****************************************************************************
 * Name: pci_disable_msi
 *
 * Description:
 *  Disable MSI.
 *
 * Input Parameters:
 *   dev  - device
 *   msi  - MSI base address
 *
 * Return value:
 *   None
 *
 ****************************************************************************/

static void pci_disable_msi(FAR struct pci_device_s *dev, uint8_t msi)
{
  uint16_t flags = 0;

  pci_read_config_word(dev, msi + PCI_MSI_FLAGS, &flags);

  flags &= ~PCI_MSI_FLAGS_ENABLE;
  pci_write_config_word(dev, msi + PCI_MSI_FLAGS, flags);
}

/****************************************************************************
 * Name: pci_enable_msix
 *
 * Description:
 *   Configure and enable MSI-X.
 *
 * Input Parameters:
 *   dev  - device
 *   irq  - allocated vectors
 *   num  - number of vectors
 *   msix - MSI-X base address
 *
 * Return value:
 *   OK on success or a negative error code on failure
 *
 ****************************************************************************/

static int pci_enable_msix(FAR struct pci_device_s *dev, FAR int *irq,
                           int num, uint8_t msix)
{
  uint32_t  mdr       = 0;
  uint16_t  flags     = 0;
  uintptr_t mar       = 0;
  uintptr_t tbladdr   = 0;
  uintptr_t tblend    = 0;
  uint32_t  tbloffset = 0;
  uint32_t  tblbar    = 0;
  uint32_t  tbl       = 0;
  uint16_t  tblsize   = 0;
  int       i         = 0;
  int       ret       = OK;

  /* Get Flags */

  pci_read_config_word(dev, msix + PCI_MSIX_FLAGS, &flags);

  /* Table Size is N - 1 encoded */

  tblsize = (flags & PCI_MSIX_FLAGS_QSIZE) + 1;

  /* Get MSI-X table */

  pci_read_config_dword(dev, msix + PCI_MSIX_TABLE, &tbl);

  /* Extract table address */

  tblbar = tbl & PCI_MSIX_TABLE_BIR;
  tbladdr = pci_resource_start(dev, tblbar);
  tbloffset = (tbl & PCI_MSIX_TABLE_OFFSET) >> PCI_MSIX_TABLE_OFFSET_SHIFT;
  tbladdr += tbloffset;

  /* Map MSI-X table */

  tblend = tbladdr + tblsize * PCI_MSIX_ENTRY_SIZE;

  if (dev->bus->ctrl->ops->map)
    {
      tbladdr = dev->bus->ctrl->ops->map(dev->bus, tbladdr, tblend);
    }

  /* Limit tblsize */

  if (num > tblsize)
    {
      pciinfo("Limit tblszie to %xu\n", tblsize);
      num = tblsize;
    }

  for (i = 0; i < num; i++)
    {
      /* Connect MSI-X (arch-specific) */

      ret = dev->bus->ctrl->ops->connect_irq(dev->bus, &irq[i], 1,
                                             &mar, &mdr);
      if (ret < 0)
        {
          return ret;
        }

      /* Write Message Address Register */

      pci_write_mmio_dword(dev, tbladdr + PCI_MSIX_ENTRY_LOWER_ADDR, mar);

      pci_write_mmio_dword(dev, tbladdr + PCI_MSIX_ENTRY_UPPER_ADDR,
                           (mar >> 32));

      /* Write Message Data Register */

      pci_write_mmio_dword(dev, tbladdr + PCI_MSIX_ENTRY_DATA, mdr);

      /* Write Vector Control register */

      pci_write_mmio_dword(dev, tbladdr + PCI_MSIX_ENTRY_VECTOR_CTRL, 0);

      /* Next vector */

      tbladdr += PCI_MSIX_ENTRY_SIZE;
    }

  /* Enable MSI-X */

  flags |= PCI_MSIX_FLAGS_ENABLE;
  pci_write_config_word(dev, msix + PCI_MSIX_FLAGS, flags);

  return OK;
}
#endif  /* CONFIG_PCI_MSIX */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_find_device_from_bus
 *
 * Description:
 *   To find a PCI device from the bus
 *
 * Input Parameters:
 *   bus   - pci bus
 *   busno - bus number
 *   devfn - device number and function number
 *
 * Returned Value:
 *   Failed if return NULL, otherwise return pci devices
 *
 ****************************************************************************/

FAR struct pci_device_s *
pci_find_device_from_bus(FAR struct pci_bus_s *bus, uint8_t busno,
                         unsigned int devfn)
{
  FAR struct pci_device_s *dev;
  int ret;

  ret = nxmutex_lock(&g_pci_lock);
  if (ret < 0)
    {
      return NULL;
    }

  dev = pci_do_find_device_from_bus(bus, busno, devfn);
  nxmutex_unlock(&g_pci_lock);

  return dev;
}

/****************************************************************************
 * Name: pci_bus_read_config
 *
 * Description:
 *  Read pci device config space
 *
 * Input Parameters:
 *   bus   - The PCI device belong to
 *   devfn - The PCI device dev number and function number
 *   where - The register address
 *   size  - The data length
 *   val   - The data buffer
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_read_config(FAR struct pci_bus_s *bus,
                        unsigned int devfn, int where,
                        int size, FAR uint32_t *val)
{
  if (size != 1 && size != 2 && size != 4)
    {
      return -EINVAL;
    }

  return bus->ctrl->ops->read(bus, devfn, where, size, val);
}

/****************************************************************************
 * Name: pci_bus_write_config
 *
 * Description:
 *  Read pci device config space
 *
 * Input Parameters:
 *   bus   - The PCI device belong to
 *   devfn - The PCI device dev number and function number
 *   where - The register address
 *   size  - The data length
 *   val   - The data
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_write_config(FAR struct pci_bus_s *bus,
                         unsigned int devfn, int where,
                         int size, uint32_t val)
{
  if (size != 1 && size != 2 && size != 4)
    {
      return -EINVAL;
    }

  return bus->ctrl->ops->write(bus, devfn, where, size, val);
}

/****************************************************************************
 * Name: pci_bus_read_io
 *
 * Description:
 *  Read pci device io space
 *
 * Input Parameters:
 *   bus   - The PCI device belong to
 *   addr  - The address to read
 *   size  - The data length
 *   val   - The data buffer
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_read_io(FAR struct pci_bus_s *bus, uintptr_t addr,
                    int size, FAR uint32_t *val)
{
  if (size != 1 && size != 2 && size != 4)
    {
      return -EINVAL;
    }

  return bus->ctrl->ops->read_io(bus, addr, size, val);
}

/****************************************************************************
 * Name: pci_bus_write_io
 *
 * Description:
 *  Read pci device io space
 *
 * Input Parameters:
 *   bus   - The PCI device belong to
 *   addr  - The address to write
 *   size  - The data length
 *   val   - The data
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_bus_write_io(FAR struct pci_bus_s *bus, uintptr_t addr,
                     int size, uint32_t val)
{
  if (size != 1 && size != 2 && size != 4)
    {
      return -EINVAL;
    }

  return bus->ctrl->ops->write_io(bus, addr, size, val);
}

/****************************************************************************
 * Name: pci_set_master
 *
 * Description:
 *   Enables bus-mastering for device
 *
 * Input Parameters:
 *   dev - The PCI device to enable
 *
 ****************************************************************************/

void pci_set_master(FAR struct pci_device_s *dev)
{
  pci_change_master(dev, true);
}

/****************************************************************************
 * Name: pci_clear_master
 *
 * Description:
 *   Disables bus-mastering for device
 *
 * Input Parameters:
 *   dev - The PCI device to disable
 *
 ****************************************************************************/

void pci_clear_master(FAR struct pci_device_s *dev)
{
  pci_change_master(dev, false);
}

/****************************************************************************
 * Name: pci_enable_device
 *
 * Description:
 *   Initialize device before it's used by a driver by setting command
 *   register.
 *
 * Input Parameters:
 *   dev - PCI device to be enabled
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_enable_device(FAR struct pci_device_s *dev)
{
  uint32_t cmd;

  pci_read_config_dword(dev, PCI_COMMAND, &cmd);
  return pci_write_config_dword(dev, PCI_COMMAND,
                                cmd | PCI_COMMAND_IO | PCI_COMMAND_MEMORY);
}

/****************************************************************************
 * Name: pci_disable_device
 *
 * Description:
 *   Disable pci device before it's unused by a driver by setting command
 *   register.
 *
 * Input Parameters:
 *   dev - PCI device to be disable
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

int pci_disable_device(FAR struct pci_device_s *dev)
{
  uint32_t cmd;

  pci_read_config_dword(dev, PCI_COMMAND, &cmd);
  return pci_write_config_dword(dev, PCI_COMMAND,
                                cmd & ~PCI_COMMAND_IO & ~PCI_COMMAND_MEMORY);
}

/****************************************************************************
 * Name: pci_select_bars
 *
 * Description:
 *   Make BAR mask from the type of resource
 *   This helper routine makes bar mask from the type of resource.
 *
 * Input Parameters:
 *   dev   - The PCI device for which BAR mask is made
 *   flags - Resource type mask to be selected
 *
 ****************************************************************************/

int pci_select_bars(FAR struct pci_device_s *dev, unsigned int flags)
{
  int bars = 0;
  int i;

  for (i = 0; i < PCI_NUM_RESOURCES; i++)
    {
      if (pci_resource_flags(dev, i) & flags)
        {
          bars |= 1 << i;
        }
    }

  return bars;
}

/****************************************************************************
 * Name: pci_bus_map_region
 *
 * Description:
 *   Create a virtual mapping for a address.
 *
 *   Using this function you will get an virtual address.
 *   These functions hide the details if this is a MMIO or PIO address
 *   space and will just do what you expect from them in the correct way.
 *
 * Input Parameters:
 *   bus   - PCI bus
 *   start - The address base
 *   size  - The length of the address
 *
 * Returned Value:
 *  Virtual address or zero if failed
 ****************************************************************************/

FAR void *
pci_bus_map_region(FAR struct pci_bus_s *bus, uintptr_t start, size_t size)
{
  return bus->ctrl->ops->map ?
    (FAR void *)bus->ctrl->ops->map(bus, start, start + size)
    : (FAR void *)start;
}

/****************************************************************************
 * Name: pci_map_bar_region
 *
 * Description:
 *   Create a virtual mapping for a PCI BAR REGION.
 *
 *   Using this function you will get an address to your device BAR region.
 *   These functions hide the details if this is a MMIO or PIO address
 *   space and will just do what you expect from them in the correct way.
 *
 * Input Parameters:
 *   dev - PCI device that owns the BAR
 *   bar - BAR number
 *   offset - BAR region offset
 *   length - BAR region length
 *
 * Returned Value:
 *  IO address or zero if failed
 ****************************************************************************/

FAR void *pci_map_bar_region(FAR struct pci_device_s *dev, int bar,
                             uintptr_t offset, size_t length)
{
  uintptr_t start = pci_resource_start(dev, bar) + offset;
  return pci_map_region(dev, start, length);
}

/****************************************************************************
 * Name: pci_map_bar
 *
 * Description:
 *   Create a virtual mapping cookie for a PCI BAR.
 *
 *   Using this function you will get an address to your device BAR.
 *   These functions hide the details if this is a MMIO or PIO address
 *   space and will just do what you expect from them in the correct way.
 *
 * Input Parameters:
 *   dev - PCI device that owns the BAR
 *   bar - BAR number
 *
 * Returned Value:
 *  IO address or zero if failed
 *
 ****************************************************************************/

FAR void *pci_map_bar(FAR struct pci_device_s *dev, int bar)
{
  return pci_map_bar_region(dev, bar, 0, pci_resource_len(dev, bar));
}

/****************************************************************************
 * Name: pci_find_capability
 *
 * Description:
 *   Query for devices' capabilities
 *
 *   Tell if a device supports a given PCI capability.
 *
 * Input Parameters:
 *   dev - PCI device to query
 *   cap - Capability code
 *
 * Returned Value:
 *   Returns the address of the requested capability structure within the
 *   device's PCI configuration space or 0 in case the device does not
 *   support it.
 *
 ****************************************************************************/

uint8_t pci_find_capability(FAR struct pci_device_s *dev, int cap)
{
  uint8_t pos;

  pos = pci_bus_find_start_cap(dev->bus, dev->devfn, dev->hdr_type);
  if (pos)
    {
      pos = pci_find_next_cap(dev->bus, dev->devfn, pos, cap);
    }

  return pos;
}

/****************************************************************************
 * Name: pci_find_next_capability
 *
 * Description:
 *   To find the next capability.
 *
 * Input Parameters:
 *   dev - The PCI device to find capbilities
 *   pos - List node
 *   cap - Value of capabilities
 *
 * Returned Value:
 *   Return the capability data
 *
 ****************************************************************************/

uint8_t pci_find_next_capability(FAR struct pci_device_s *dev, uint8_t pos,
                                 int cap)
{
  return pci_find_next_cap(dev->bus, dev->devfn,
                           pos + PCI_CAP_LIST_NEXT, cap);
}

/****************************************************************************
 * Name: pci_stat_line
 *
 * Description:
 *  Determine if the interrupt line is active for a given device
 *
 * Input Parameters:
 *   dev - device
 *
 * Return value:
 *   True if interrupt is active
 *
 ****************************************************************************/

bool pci_stat_line(FAR struct pci_device_s *dev)
{
  uint16_t tmp1;
  uint16_t tmp2;

  /* Interrupts enabled if Interrupt Disable is not set and Interrupt Status
   * is set.
   */

  pci_read_config_word(dev, PCI_COMMAND, &tmp1);
  pci_read_config_word(dev, PCI_STATUS, &tmp2);

  return (!(tmp1 & PCI_COMMAND_INTX_DISABLE) &&
          (tmp2 & PCI_STATUS_INTERRUPT));
}

/****************************************************************************
 * Name: pci_get_irq
 *
 * Description:
 *  Get interrupt number associated with a device PCI interrupt line
 *
 * Input Parameters:
 *   dev - PCI device
 *
 * Return value:
 *   Return interrupt number associated with a given INTx.
 *
 ****************************************************************************/

int pci_get_irq(FAR struct pci_device_s *dev)
{
  uint8_t line = 0;
  uint8_t pin = 0;

  pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &line);
  pci_read_config_byte(dev, PCI_INTERRUPT_PIN, &pin);

  if (dev->bus->ctrl->ops->get_irq)
    {
      return dev->bus->ctrl->ops->get_irq(dev->bus, dev->devfn, line, pin);
    }

  return -ENOTSUP;
}

/****************************************************************************
 * Name: pci_alloc_irq
 *
 * Description:
 *   Allocate MSI or MSI-X vectors
 *
 * Input Parameters:
 *   dev - PCI device
 *   irq - allocated vectors
 *   num - number of vectors
 *
 * Return value:
 *   Return the number of allocated vectors on succes or negative errno
 *   on failure.
 *
 ****************************************************************************/

int pci_alloc_irq(FAR struct pci_device_s *dev, FAR int *irq, int num)
{
  if (dev->bus->ctrl->ops->alloc_irq)
    {
      return dev->bus->ctrl->ops->alloc_irq(dev->bus, irq, num);
    }

  return -ENOTSUP;
}

/****************************************************************************
 * Name: pci_release_irq
 *
 * Description:
 *   Release MSI or MSI-X vectors
 *
 * Input Parameters:
 *   dev - PCI device
 *   irq - allocated vectors
 *   num - number of vectors
 *
 * Return value:
 *   Failed if return a negative value, otherwise success
 *
 ****************************************************************************/

void pci_release_irq(FAR struct pci_device_s *dev, FAR int *irq, int num)
{
  if (dev->bus->ctrl->ops->release_irq)
    {
      dev->bus->ctrl->ops->release_irq(dev->bus, irq, num);
    }
}

/****************************************************************************
 * Name: pci_connect_irq
 *
 * Description:
 *   Connect MSI or MSI-X if available.
 *
 * Input Parameters:
 *   dev - PCI device
 *   irq - allocated vectors
 *   num - number of vectors
 *
 * Return value:
 *   Return -ENOSETUP if MSI/MSI-X not available. Return OK on success.
 *
 ****************************************************************************/

int pci_connect_irq(FAR struct pci_device_s *dev, FAR int *irq, int num)
{
  uint8_t msi = 0;
  uint8_t msix = 0;

  if (dev->bus->ctrl->ops->connect_irq == NULL)
    {
      return -ENOTSUP;
    }

  /* Get MSI base */

  pci_get_msi_base(dev, &msi, &msix);
  if (msi == 0 && msix == 0)
    {
      /* MSI and MSI-X not supported */

      return -ENOTSUP;
    }

  /* Configure MSI or MSI-X */

#ifdef CONFIG_PCI_MSIX
  if (msix != 0)
    {
      /* Disalbe MSI */

      pci_disable_msi(dev, msi);

      /* Enable MSI-X */

      return pci_enable_msix(dev, irq, num, msix);
    }
  else
#endif
    {
      /* Enable MSI */

      return pci_enable_msi(dev, irq, num, msi);
    }
}

/****************************************************************************
 * Name: pci_register_driver
 *
 * Description:
 *   To register a PCI driver
 *
 * Input Parameters:
 *   drv - PCI driver
 *
 * Returned Value:
 *   Failed if return a negative value, otherwise success
 *
 ****************************************************************************/

int pci_register_driver(FAR struct pci_driver_s *drv)
{
  FAR const struct pci_device_id_s *id;
  FAR struct pci_device_s *dev;
  int ret;

  DEBUGASSERT(drv != NULL && drv->probe != NULL && drv->id_table != NULL);

  ret = nxmutex_lock(&g_pci_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Add the driver to the pci driver list */

  list_add_tail(&g_pci_driver_list, &drv->node);

  list_for_every_entry(&g_pci_device_list, dev, struct pci_device_s, node)
    {
      if (dev->drv != NULL)
        {
          continue;
        }

      for (id = drv->id_table; id->vendor; id++)
        {
          if (pci_match_one_device(id, dev))
            {
              dev->id = id;
              if (drv->probe(dev) >= 0)
                {
                  dev->drv = drv;
                }
            }
        }
    }

  nxmutex_unlock(&g_pci_lock);
  return ret;
}

/****************************************************************************
 * Name: pci_unregister_driver
 *
 * Description:
 *   To unregister a PCI driver
 *
 * Input Parameters:
 *   drv - PCI driver
 *
 * Returned Value:
 *   Failed if return a negative value, otherwise success
 *
 ****************************************************************************/

int pci_unregister_driver(FAR struct pci_driver_s *drv)
{
  FAR struct pci_device_s *dev;
  int ret;

  DEBUGASSERT(drv != NULL && drv->remove != NULL);

  ret = nxmutex_lock(&g_pci_lock);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry(&g_pci_device_list, dev, struct pci_device_s, node)
    {
      if (dev->drv == drv)
        {
          drv->remove(dev);
          dev->drv = NULL;
        }
    }

  list_delete(&drv->node);

  nxmutex_unlock(&g_pci_lock);
  return ret;
}

/****************************************************************************
 * Name: pci_register_device
 *
 * Description:
 *   To register a PCI device
 *
 * Input Parameters:
 *   dev - PCI device
 *
 * Returned Value:
 *   Failed if return a negative value, otherwise success
 *
 ****************************************************************************/

int pci_register_device(FAR struct pci_device_s *dev)
{
  FAR const struct pci_device_id_s *id;
  FAR struct pci_driver_s *drv;
  int ret;

  ret = nxmutex_lock(&g_pci_lock);
  if (ret < 0)
    {
      return ret;
    }

  list_add_tail(&g_pci_device_list, &dev->node);

  list_for_every_entry(&g_pci_driver_list, drv, struct pci_driver_s, node)
    {
      for (id = drv->id_table; id->vendor; id++)
        {
          if (pci_match_one_device(id, dev))
            {
              dev->id = id;
              if (drv->probe(dev) >= 0)
                {
                  dev->drv = drv;
                  goto out;
                }
            }
        }
    }

out:
  nxmutex_unlock(&g_pci_lock);
  return ret;
}

/****************************************************************************
 * Name: pci_unregister_device
 *
 * Description:
 *   To unregister a PCI device
 *
 * Input Parameters:
 *   dev - PCI device
 *
 * Returned Value:
 *   Failed if return a negative value, otherwise success
 *
 ****************************************************************************/

int pci_unregister_device(FAR struct pci_device_s *dev)
{
  int ret;

  ret = nxmutex_lock(&g_pci_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->drv && dev->drv->remove)
    {
      dev->drv->remove(dev);
    }

  dev->drv = NULL;
  list_delete(&dev->node);

  nxmutex_unlock(&g_pci_lock);
  return ret;
}

/****************************************************************************
 * Name: pci_register_controller
 *
 * Description:
 *   Start pci bridge enumeration process, and register pci device.
 *
 * Input Parameters:
 *   ctrl - PCI controller to register
 *
 ****************************************************************************/

int pci_register_controller(FAR struct pci_controller_s *ctrl)
{
  FAR struct pci_bus_s *bus;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  bus = pci_alloc_bus();
  if (bus == NULL)
    {
      return -ENOMEM;
    }

  bus->ctrl = ctrl;

  ctrl->bus = bus;
  ctrl->busno = 1;

  pci_scan_bus(bus);
  pci_register_bus_devices(bus);

  nxmutex_lock(&g_pci_lock);
  list_add_tail(&g_pci_ctrl_list, &ctrl->node);
  nxmutex_unlock(&g_pci_lock);

  return 0;
}

/****************************************************************************
 * Name: pci_bus_find_capability
 *
 * Description:
 *   Query for devices' capabilities
 *
 *   Tell if a device supports a given PCI capability.
 *
 * Input Parameters:
 *   bus    - PCI device bus belong to
 *   devfn  - PCI device number and function number
 *   cap    - Capability code
 *
 * Returned Value:
 *   Returns the address of the requested capability structure within the
 *   device's PCI configuration space or 0 in case the device does not
 *   support it.
 *
 ****************************************************************************/

uint8_t pci_bus_find_capability(FAR struct pci_bus_s *bus,
                                unsigned int devfn, int cap)
{
  uint8_t type = 0;
  uint8_t pos;

  pci_bus_read_config_byte(bus, devfn, PCI_HEADER_TYPE, &type);
  pos = pci_bus_find_start_cap(bus, devfn, type);
  if (pos)
    {
      pos = pci_find_next_cap(bus, devfn, pos, cap);
    }

  return pos;
}

/****************************************************************************
 * Name: pci_dev_register
 *
 * Description:
 *   Create an pci dev driver.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

int pci_dev_register(void)
{
  return register_driver("/dev/pci", &g_pci_fops, 0666, NULL);
}

/****************************************************************************
 * Name: pci_bus_read_config_xxx
 *
 * Description:
 *  Read pci device config space
 *
 * Input Parameters:
 *   bus   - The PCI device to belong to
 *   devfn - The PCI device number and function number
 *   where - The register address
 *   val   - The data buf
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

PCI_BUS_READ_CONFIG(byte, uint8_t, 1)
PCI_BUS_READ_CONFIG(word, uint16_t, 2)
PCI_BUS_READ_CONFIG(dword, uint32_t, 4)

/****************************************************************************
 * Name: pci_bus_write_config_xxx
 *
 * Description:
 *  Write pci device config space
 *
 * Input Parameters:
 *   bus   - The PCI device to belong to
 *   devfn - The PCI device number and function number
 *   where - The register address
 *   val   - The data
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

PCI_BUS_WRITE_CONFIG(byte, uint8_t, 1)
PCI_BUS_WRITE_CONFIG(word, uint16_t, 2)
PCI_BUS_WRITE_CONFIG(dword, uint32_t, 4)

/****************************************************************************
 * Name: pci_bus_read_io_xxx
 *
 * Description:
 *  Read pci device io space
 *
 * Input Parameters:
 *   bus   - The PCI device belong to
 *   where - The address to read
 *   val   - The data buffer
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

PCI_BUS_READ_IO(byte, uint8_t, 1)
PCI_BUS_READ_IO(word, uint16_t, 2)
PCI_BUS_READ_IO(dword, uint32_t, 4)

/****************************************************************************
 * Name: pci_bus_write_io_xxx
 *
 * Description:
 *  Write pci device io space
 *
 * Input Parameters:
 *   bus   - The PCI device belong to
 *   where - The address to write
 *   val   - The data
 *
 * Returned Value:
 *   Zero if success, otherwise nagative
 *
 ****************************************************************************/

PCI_BUS_WRITE_IO(byte, uint8_t, 1)
PCI_BUS_WRITE_IO(word, uint16_t, 2)
PCI_BUS_WRITE_IO(dword, uint32_t, 4)
