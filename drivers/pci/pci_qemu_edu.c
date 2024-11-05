/****************************************************************************
 * drivers/pci/pci_qemu_edu.c
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

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pci/pci.h>
#include <nuttx/semaphore.h>

#include "pci_drivers.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCI_QEMU_EDU_CONTROL_BAR_ID 0

/* Registers defined for device.  Size 4 for < 0x80.  Size 8 for >= 0x80. */

#define PCI_QEMU_EDU_REG_ID         0x00  /* Identification */
#define PCI_QEMU_EDU_REG_LIVE       0x04  /* Liveness Check */
#define PCI_QEMU_EDU_REG_FAC        0x08  /* Factorial Computation */
#define PCI_QEMU_EDU_REG_STATUS     0x20  /* Status */
#define PCI_QEMU_EDU_REG_INT_STATUS 0x24  /* Interupt Status */
#define PCI_QEMU_EDU_REG_INT_RAISE  0x60  /* Raise an interrupt */
#define PCI_QEMU_EDU_REG_INT_ACK    0x64  /* Acknowledge interrupt */
#define PCI_QEMU_EDU_REG_DMA_SOURCE 0x80  /* Source address for DMA transfer */
#define PCI_QEMU_EDU_REG_DMA_DEST   0x88  /* Destination address for DMA transfer */
#define PCI_QEMU_EDU_REG_DMA_COUNT  0x90  /* Size of area to transfer with DMA */
#define PCI_QEMU_EDU_REG_DMA_CMD    0x98  /* Control DMA tranfer */

/* One 4096 bytes long buffer at offset 0x40000 is available in the
 * EDU device
 */

#define PCI_QEMU_EDU_DMABUF_OFFSET  0x40000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pci_qemu_edu_priv_s
{
  FAR struct pci_device_s *dev;
  uintptr_t                base_addr;
  sem_t                    isr_done;
  uint32_t                 test_result;
};

/****************************************************************************
 * Private Functions Definitions
 ****************************************************************************/

static uint32_t pci_qemu_edu_read_reg32(FAR struct pci_qemu_edu_priv_s *priv,
                                        int reg);
static void pci_qemu_edu_write_reg32(FAR struct pci_qemu_edu_priv_s *priv,
                                     int reg, uint32_t val);
static void pci_qemu_edu_write_reg64(FAR struct pci_qemu_edu_priv_s *priv,
                                     int reg, uint64_t val);

static void pci_qemu_edu_test_poll(FAR struct pci_qemu_edu_priv_s *priv);
static void pci_qemu_edu_test_intx(FAR struct pci_qemu_edu_priv_s *priv);
static int pci_qemu_edu_interrupt(int irq, FAR void *context, FAR void *arg);

static int pci_qemu_edu_probe(FAR struct pci_device_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_device_id_s g_pci_qemu_edu_id_table[] =
{
  { PCI_DEVICE(0x1234, 0x11e8), },
  { }
};

static struct pci_driver_s g_pci_qemu_edu_pci_drv =
{
  .id_table = g_pci_qemu_edu_id_table,
  .probe    = pci_qemu_edu_probe,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_qemu_edu_read_reg32
 *
 * Description:
 *   Provide a read interface for 32bit mapped registers
 *
 * Input Parameters:
 *   priv - Edu driver private data
 *   reg  - Register offset
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

static uint32_t pci_qemu_edu_read_reg32(FAR struct pci_qemu_edu_priv_s *priv,
                                        int reg)
{
  return *(FAR volatile uint32_t *)(priv->base_addr + reg);
}

/****************************************************************************
 * Name: pci_qemu_edu_write_reg32
 *
 * Description:
 *   Provide a write interface for 32bit mapped registers
 *
 * Input Parameters:
 *   priv - Edu driver private data
 *   reg  - Register offset
 *   val  - Value to assign to register
 *
 ****************************************************************************/

static void pci_qemu_edu_write_reg32(FAR struct pci_qemu_edu_priv_s *priv,
                                     int reg, uint32_t val)
{
  *(FAR volatile uint32_t *)(priv->base_addr + reg) = val;
}

/****************************************************************************
 * Name: pci_qemu_edu_write_reg64
 *
 * Description:
 *   Provide a write interface for 64bit mapped registers
 *
 * Input Parameters:
 *   priv - Edu driver private data
 *   reg  - Register offset
 *   val  - Value to assign to register
 *
 ****************************************************************************/

static void pci_qemu_edu_write_reg64(FAR struct pci_qemu_edu_priv_s *priv,
                                     int reg, uint64_t val)
{
  *(FAR volatile uint64_t *)(priv->base_addr + reg) = val;
}

/****************************************************************************
 * Name: pci_qemu_edu_test_poll
 *
 * Description:
 *   Performs basic functional test of PCI device and MMIO using polling
 *   of mapped register interfaces.
 *
 * Input Parameters:
 *   priv - Edu driver private data
 *
 ****************************************************************************/

static void pci_qemu_edu_test_poll(FAR struct pci_qemu_edu_priv_s *priv)
{
  uint32_t test_value;
  uint32_t test_read;

  pciinfo("Identification: 0x%08" PRIx32 "u\n",
          pci_qemu_edu_read_reg32(priv, PCI_QEMU_EDU_REG_ID));

  /* Test Live Check */

  test_value = 0xdeadbeef;
  pci_qemu_edu_write_reg32(priv, PCI_QEMU_EDU_REG_LIVE, test_value);
  test_read = pci_qemu_edu_read_reg32(priv, PCI_QEMU_EDU_REG_LIVE);
  pciinfo("Live Check: Wrote: 0x%08" PRIx32 " Read: 0x%08" PRIx32
          " Error Bits 0x%08" PRIx32 "\n",
          test_value, test_read, test_read ^ ~test_value);
  pciinfo("TEST %s\n", ((test_read ^ ~test_value) == 0) ? "PASS" : "FAIL");

  /* Test Factorial */

  test_value = 10;
  pci_qemu_edu_write_reg32(priv, PCI_QEMU_EDU_REG_STATUS, 0);
  pci_qemu_edu_write_reg32(priv, PCI_QEMU_EDU_REG_FAC, test_value);
  while (pci_qemu_edu_read_reg32(priv, PCI_QEMU_EDU_REG_STATUS) & 0x01)
    {
      pciinfo("Waiting to compute factorial...");
      usleep(10000);
    }

  test_read = pci_qemu_edu_read_reg32(priv, PCI_QEMU_EDU_REG_FAC);
  pciinfo("Computed factorial of %" PRIu32 " as %" PRIu32 "\n",
          test_value, test_read);
  pciinfo("TEST %s\n", (test_read == 3628800) ? "PASS" : "FAIL");
}

/****************************************************************************
 * Name: pci_qemu_edu_test_intx
 *
 * Description:
 *   Performs basic functional test of PCI device and MMIO using INTx
 *
 * Input Parameters:
 *   priv  - Struct containing internal state of driver
 *
 ****************************************************************************/

static void pci_qemu_edu_test_intx(FAR struct pci_qemu_edu_priv_s *priv)
{
  uint32_t test_value;

  pciinfo("Identification: 0x%08" PRIx32 "u\n",
          pci_qemu_edu_read_reg32(priv, PCI_QEMU_EDU_REG_ID));

  /* Test Read/Write */

  test_value = 0xdeadbeef;
  pciinfo("Triggering interrupt with value 0x%08" PRIx32 "\n", test_value);
  pci_qemu_edu_write_reg32(priv, PCI_QEMU_EDU_REG_INT_RAISE, test_value);
  nxsem_wait(&priv->isr_done);
  pciinfo("TEST %s\n", (priv->test_result == test_value) ? "PASS" : "FAIL");

  /* Test Factorial */

  test_value = 5;
  pciinfo("Computing factorial of %" PRIu32 "\n", test_value);
  pci_qemu_edu_write_reg32(priv, PCI_QEMU_EDU_REG_STATUS, 0x80);
  pci_qemu_edu_write_reg32(priv, PCI_QEMU_EDU_REG_FAC, test_value);
  nxsem_wait(&priv->isr_done);
  pciinfo("TEST %s\n", (priv->test_result == 120) ? "PASS" : "FAIL");

  /* Test ISR Status Cleanup */

  pci_qemu_edu_write_reg32(priv, PCI_QEMU_EDU_REG_INT_RAISE, test_value);
  nxsem_wait(&priv->isr_done);
  pciinfo("TEST %s\n", (priv->test_result == test_value) ?
                       "PASS" : "FAIL");
}

/****************************************************************************
 * Name: pci_qemu_edu_test_dma
 *
 * Description:
 *   Performs dma functional test of PCI device
 *
 * Input Parameters:
 *   priv - Struct containing internal state of driver
 *
 ****************************************************************************/

static void pci_qemu_edu_test_dma(FAR struct pci_qemu_edu_priv_s *priv)
{
  const uint64_t dev_addr = PCI_QEMU_EDU_DMABUF_OFFSET;
  const size_t block_size = 2048;
  FAR void *test_block;
  uint32_t tx_checksum;
  uint32_t rx_checksum;
  uint32_t psrand;
  int i;

  pciinfo("Identification: 0x%08" PRIx32 "u\n",
          pci_qemu_edu_read_reg32(priv, PCI_QEMU_EDU_REG_ID));

  test_block = kmm_malloc(block_size);
  for (i = 0; i < block_size; i++)
    {
      *((FAR uint8_t *)test_block + i) = i & 0xff;
    }

  tx_checksum = 0;
  psrand = 0x0011223344;
  for (i = 0; i < block_size / 4; i++)
    {
      /* Fill the memory block with "random" data */

      psrand ^= psrand << 13;
      psrand ^= psrand >> 17;
      psrand ^= psrand << 5;
      *((FAR uint32_t *)test_block + i) = psrand;
      tx_checksum += psrand;
    }

  pciinfo("Test block checksum 0x%08" PRIx32 "\n", tx_checksum);
  pci_qemu_edu_write_reg64(priv, PCI_QEMU_EDU_REG_DMA_SOURCE,
                           (uint64_t)test_block);
  pci_qemu_edu_write_reg64(priv, PCI_QEMU_EDU_REG_DMA_DEST, dev_addr);
  pci_qemu_edu_write_reg64(priv, PCI_QEMU_EDU_REG_DMA_COUNT,
                           (uint64_t)block_size);
  pci_qemu_edu_write_reg32(priv, PCI_QEMU_EDU_REG_STATUS, 0x00);
  pci_qemu_edu_write_reg64(priv, PCI_QEMU_EDU_REG_DMA_CMD, 0x01 | 0x04);
  nxsem_wait(&priv->isr_done);

  pciinfo("DMA transfer to device complete.\n");

  pci_qemu_edu_write_reg64(priv, PCI_QEMU_EDU_REG_DMA_DEST,
                           (uint64_t)test_block);
  pci_qemu_edu_write_reg64(priv, PCI_QEMU_EDU_REG_DMA_SOURCE, dev_addr);
  pci_qemu_edu_write_reg64(priv, PCI_QEMU_EDU_REG_DMA_COUNT,
                           (uint64_t)block_size);
  pci_qemu_edu_write_reg32(priv, PCI_QEMU_EDU_REG_STATUS, 0x00);
  pci_qemu_edu_write_reg64(priv, PCI_QEMU_EDU_REG_DMA_CMD,
                           0x01 | 0x02 | 0x04);
  nxsem_wait(&priv->isr_done);

  pciinfo("DMA transfer from device complete.\n");
  rx_checksum = 0;
  for (i = 0; i < block_size / 4; i++)
    {
      rx_checksum += *((FAR uint32_t *)test_block + i);
    }

  kmm_free(test_block);

  pciinfo("Received block checksum 0x%08" PRIx32 "\n", rx_checksum);
  pciinfo("TEST %s\n", (rx_checksum == tx_checksum) ? "PASS" : "FAIL");
}

/****************************************************************************
 * Name: pci_qemu_edu_interrupt
 *
 * Description:
 *  EDU interrupt handler
 *
 ****************************************************************************/

static int pci_qemu_edu_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct pci_qemu_edu_priv_s *priv =
    (FAR struct pci_qemu_edu_priv_s *)arg;
  uint32_t status;

  status = pci_qemu_edu_read_reg32(priv, PCI_QEMU_EDU_REG_INT_STATUS);
  pci_qemu_edu_write_reg32(priv, PCI_QEMU_EDU_REG_INT_ACK, ~0u);
  switch (status)
    {
      /* Factorial triggered */

      case 0x1:
        {
          priv->test_result =
            pci_qemu_edu_read_reg32(priv, PCI_QEMU_EDU_REG_FAC);
          pciinfo("Computed factorial: %" PRIu32 "\n", priv->test_result);
          break;
        }

      /* DMA triggered */

      case 0x100:
        {
          pciinfo("DMA transfer complete\n");
          break;
        }

      /* Generic write */

      default:
        {
          priv->test_result = status;
          pciinfo("Received value: 0x%08" PRIx32 "\n", status);
          break;
        }
    }

  nxsem_post(&priv->isr_done);
  return OK;
}

/****************************************************************************
 * Name: pci_qemu_edu_probe
 *
 * Description:
 *   Initialize device
 *
 ****************************************************************************/

static int pci_qemu_edu_probe(FAR struct pci_device_s *dev)
{
  struct pci_qemu_edu_priv_s priv;
  unsigned int flags;
  int irq = 0;
  int ret;

  /* Enable EDU device */

  ret = pci_enable_device(dev);
  if (ret < 0)
    {
      pcierr("Enable device failed, ret=%d\n", ret);
      return ret;
    }

  pci_set_master(dev);

  /* Initialize the edu driver */

  pciinfo("EDU Device Init\n");

  priv.dev = dev;
  flags = pci_resource_flags(dev, PCI_QEMU_EDU_CONTROL_BAR_ID);
  if ((flags & PCI_RESOURCE_MEM) != PCI_RESOURCE_MEM)
    {
      ret = -ENODEV;
      pcierr("Control bar expected to be MMIO, flags=0x%x\n", flags);
      goto err;
    }

  priv.base_addr = (uintptr_t)pci_map_bar(dev, PCI_QEMU_EDU_CONTROL_BAR_ID);
  if (priv.base_addr == 0)
    {
      ret = -ENOMEM;
      pcierr("Control BAR is not valid\n");
      goto err;
    }

  nxsem_init(&priv.isr_done, 0, 0);

  /* Run Poll Tests */

  pciinfo("POLL TEST\n");

  pci_qemu_edu_test_poll(&priv);

  /* Run IRQ Tests */

  irq = pci_get_irq(dev);
  pciinfo("IRQ TEST: Attaching IRQ %u to %p\n", irq, pci_qemu_edu_interrupt);

  irq_attach(irq, pci_qemu_edu_interrupt, &priv);
  up_enable_irq(irq);

  pci_qemu_edu_test_intx(&priv);
  pci_qemu_edu_test_dma(&priv);

  up_disable_irq(irq);
  irq_detach(irq);

  /* Run MSI Tests */

  pciinfo("MSI TEST\n");

  irq = 0;
  ret = pci_alloc_irq(dev, &irq, 1);
  if (ret != 1)
    {
      pcierr("Failed to allocate MSI %d\n", ret);
      goto err;
    }

  pciinfo("MSI TEST: Attaching MSI %u to %p\n",
          irq, pci_qemu_edu_interrupt);

  ret = pci_connect_irq(dev, &irq, 1);
  if (ret != OK)
    {
      pcierr("Failed to connect MSI %d\n", ret);
      goto err;
    }

  irq_attach(irq, pci_qemu_edu_interrupt, &priv);
  up_enable_irq(irq);

  pci_qemu_edu_test_intx(&priv);
  pci_qemu_edu_test_dma(&priv);

  up_disable_irq(irq);
  irq_detach(irq);
  pci_release_irq(dev, &irq, 1);

  /* Uninitialize the driver */

  nxsem_destroy(&priv.isr_done);

  /* TODO: add pci unmap api */

err:
  if (irq != 0)
    {
      pci_release_irq(dev, &irq, 1);
    }

  pci_clear_master(dev);
  pci_disable_device(dev);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_register_qemu_edu_driver
 *
 * Description:
 *   Register a pci driver
 *
 ****************************************************************************/

int pci_register_qemu_edu_driver(void)
{
  return pci_register_driver(&g_pci_qemu_edu_pci_drv);
}
