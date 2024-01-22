/*****************************************************************************
 * drivers/virt/qemu_edu.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <debug.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>

#include <nuttx/pci/pci.h>
#include <nuttx/virt/qemu_pci.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Registers defined for device.  Size 4 for < 0x80.  Size 8 for >= 0x80. */

#define EDU_REG_ID          0x00  /* Identification */
#define EDU_REG_LIVE        0x04  /* Liveness Check */
#define EDU_REG_FAC         0x08  /* Factorial Computation */
#define EDU_REG_STATUS      0x20  /* Status */
#define EDU_REG_INT_STATUS  0x24  /* Interupt Status */
#define EDU_REG_INT_RAISE   0x60  /* Raise an interrupt */
#define EDU_REG_INT_ACK     0x64  /* Acknowledge interrupt */
#define EDU_REG_DMA_SOURCE  0x80  /* Source address for DMA transfer */
#define EDU_REG_DMA_DEST    0x88  /* Destination address for DMA transfer */
#define EDU_REG_DMA_COUNT   0x90  /* Size of area to transfer with DMA */
#define EDU_REG_DMA_CMD     0x98  /* Control DMA tranfer */

#define EDU_CONTROL_BAR_ID      0
#define EDU_CONTROL_BAR_OFFSET  PCI_HEADER_NORM_BAR0

/* One 4096 bytes long buffer at offset 0x40000 is available in the
 * EDU device
 */

#define QEMU_EDU_DMABUF_OFFSET 0x40000

/*****************************************************************************
 * Private Types
 *****************************************************************************/

struct qemu_edu_priv_s
{
  uintptr_t base_addr;
  sem_t     isr_done;
  uint32_t  test_result;
};

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

static void qemu_edu_write_reg32(uintptr_t addr, uint32_t val);

static uint32_t qemu_edu_read_reg32(uintptr_t addr);

static void qemu_edu_write_reg64(uintptr_t addr, uint64_t val);

static void qemu_edu_test_poll(FAR struct pci_dev_s *dev,
                               uintptr_t base_addr);

static void qemu_edu_test_intx(FAR struct pci_dev_s *dev,
                               struct qemu_edu_priv_s *drv_priv);

static int qemu_edu_interrupt(int irq, void *context, FAR void *arg);

static int qemu_edu_probe(FAR struct pci_bus_s *bus,
                          FAR const struct pci_dev_type_s *type,
                          uint16_t bdf);

/*****************************************************************************
 * Public Data
 *****************************************************************************/

const struct pci_dev_type_s g_pci_type_qemu_edu =
{
  .vendor    = 0x1234,
  .device    = 0x11e8,
  .class_rev = PCI_ID_ANY,
  .name      = "Qemu PCI EDU device",
  .probe     = qemu_edu_probe
};

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: qemu_edu_write_reg32
 *
 * Description:
 *   Provide a write interface for 32bit mapped registers
 *
 * Input Parameters:
 *   addr - Register address
 *   val  - Value to assign to register
 *
 *****************************************************************************/

static void qemu_edu_write_reg32(uintptr_t addr, uint32_t val)
{
  *(volatile uint32_t *)addr = val;
}

/*****************************************************************************
 * Name: qemu_edu_read_reg32
 *
 * Description:
 *   Provide a read interface for 32bit mapped registers
 *
 * Returned Value:
 *   Register value
 *
 *****************************************************************************/

static uint32_t qemu_edu_read_reg32(uintptr_t addr)
{
  return *(volatile uint32_t *)addr;
}

/*****************************************************************************
 * Name: qemu_edu_write_reg64
 *
 * Description:
 *   Provide a write interface for 64bit mapped registers
 *
 * Input Parameters:
 *   addr - Register address
 *   val  - Value to assign to register
 *
 *****************************************************************************/

static void qemu_edu_write_reg64(uintptr_t addr, uint64_t val)
{
  *(volatile uint64_t *)addr = val;
}

/*****************************************************************************
 * Name: qemu_edu_test_poll
 *
 * Description:
 *   Performs basic functional test of PCI device and MMIO using polling
 *   of mapped register interfaces.
 *
 * Input Parameters:
 *   bus       - An PCI device
 *   base_addr - Base address of device register space
 *
 *****************************************************************************/

static void qemu_edu_test_poll(FAR struct pci_dev_s *dev, uintptr_t base_addr)
{
  uint32_t test_value;
  uint32_t test_read;

  pciinfo("Identification: 0x%08xu\n",
          qemu_edu_read_reg32(base_addr + EDU_REG_ID));

  /* Test Live Check */

  test_value = 0xdeadbeef;
  qemu_edu_write_reg32(base_addr + EDU_REG_LIVE, test_value);
  test_read = qemu_edu_read_reg32(base_addr + EDU_REG_LIVE);
  pciinfo("Live Check: Wrote: 0x%08x Read: 0x%08x Error Bits 0x%08x\n",
          test_value, test_read, test_read ^ ~test_value);
  pciinfo("TEST %s\n", ((test_read ^ ~test_value) == 0) ? "PASS" : "FAIL");

  /* Test Factorial */

  test_value = 10;
  qemu_edu_write_reg32(base_addr + EDU_REG_STATUS, 0);
  qemu_edu_write_reg32(base_addr + EDU_REG_FAC, test_value);
  while (qemu_edu_read_reg32(base_addr + EDU_REG_STATUS) & 0x01)
    {
      pciinfo("Waiting to compute factorial...");
      usleep(10000);
    }

  test_read = qemu_edu_read_reg32(base_addr + EDU_REG_FAC);
  pciinfo("Computed factorial of %d as %d\n", test_value, test_read);
  pciinfo("TEST %s\n", (test_read == 3628800) ? "PASS" : "FAIL");
}

/*****************************************************************************
 * Name: qemu_edu_test_intx
 *
 * Description:
 *   Performs basic functional test of PCI device and MMIO using INTx
 *
 * Input Parameters:
 *   bus       - An PCI device
 *   drv_priv  - Struct containing internal state of driver
 *
 *****************************************************************************/

static void qemu_edu_test_intx(FAR struct pci_dev_s *dev,
                               FAR struct qemu_edu_priv_s *drv_priv)
{
  uintptr_t base_addr = drv_priv->base_addr;
  uint32_t  test_value;

  pciinfo("Identification: 0x%08xu\n",
          qemu_edu_read_reg32(base_addr + EDU_REG_ID));

  /* Test Read/Write */

  test_value = 0xdeadbeef;
  pciinfo("Triggering interrupt with value 0x%08x\n", test_value);
  qemu_edu_write_reg32(base_addr + EDU_REG_INT_RAISE, test_value);
  sem_wait(&drv_priv->isr_done);
  pciinfo("TEST %s\n",
      (drv_priv->test_result == test_value) ? "PASS" : "FAIL");

  /* Test Factorial */

  test_value = 5;
  pciinfo("Computing factorial of %d\n", test_value);
  qemu_edu_write_reg32(base_addr + EDU_REG_STATUS, 0x80);
  qemu_edu_write_reg32(base_addr + EDU_REG_FAC, test_value);
  sem_wait(&drv_priv->isr_done);
  pciinfo("TEST %s\n", (drv_priv->test_result == 120) ? "PASS" : "FAIL");

  /* Test ISR Status Cleanup */

  qemu_edu_write_reg32(base_addr + EDU_REG_INT_RAISE, test_value);
  sem_wait(&drv_priv->isr_done);
  pciinfo("TEST %s\n",
      (drv_priv->test_result == test_value) ? "PASS" : "FAIL");
}

/*****************************************************************************
 * Name: qemu_edu_test_dma
 *
 * Description:
 *   Performs dma functional test of PCI device
 *
 * Input Parameters:
 *   bus       - An PCI device
 *   drv_priv  - Struct containing internal state of driver
 *
 *****************************************************************************/

static void qemu_edu_test_dma(FAR struct pci_dev_s *dev,
                              FAR struct qemu_edu_priv_s *drv_priv)
{
  uintptr_t  base_addr  = drv_priv->base_addr;
  FAR void  *test_block;
  size_t     block_size = 2048;
  int        i;
  uint32_t   psrand;
  uint32_t   tx_checksum;
  uint32_t   rx_checksum;
  uint32_t   dev_addr   = QEMU_EDU_DMABUF_OFFSET;

  pciinfo("Identification: 0x%08xu\n",
          qemu_edu_read_reg32(base_addr + EDU_REG_ID));

  test_block = kmm_malloc(block_size);
  for (i = 0; i < block_size; i++)
    {
      *((uint8_t *)test_block + i) = i & 0xff;
    }

  tx_checksum = 0;
  psrand = 0x0011223344;
  for (i = 0; i < (block_size / 4); i++)
    {
      /* Fill the memory block with "random" data */

      psrand ^= psrand << 13;
      psrand ^= psrand >> 17;
      psrand ^= psrand << 5;
      *((uint32_t *)test_block + i) = psrand;
      tx_checksum += psrand;
    }

  pciinfo("Test block checksum 0x%08x\n", tx_checksum);
  qemu_edu_write_reg64(base_addr + EDU_REG_DMA_SOURCE, (uint64_t)test_block);
  qemu_edu_write_reg64(base_addr + EDU_REG_DMA_DEST, (uint64_t)dev_addr);
  qemu_edu_write_reg64(base_addr + EDU_REG_DMA_COUNT, (uint64_t)block_size);
  qemu_edu_write_reg32(base_addr + EDU_REG_STATUS, 0x00);
  qemu_edu_write_reg64(base_addr + EDU_REG_DMA_CMD, 0x01 | 0x04);
  sem_wait(&drv_priv->isr_done);

  pciinfo("DMA transfer to device complete.\n");

  qemu_edu_write_reg64(base_addr + EDU_REG_DMA_DEST, (uint64_t)test_block);
  qemu_edu_write_reg64(base_addr + EDU_REG_DMA_SOURCE, (uint64_t)dev_addr);
  qemu_edu_write_reg64(base_addr + EDU_REG_DMA_COUNT, (uint64_t)block_size);
  qemu_edu_write_reg32(base_addr + EDU_REG_STATUS, 0x00);
  qemu_edu_write_reg64(base_addr + EDU_REG_DMA_CMD, 0x01 | 0x02 | 0x04);
  sem_wait(&drv_priv->isr_done);

  pciinfo("DMA transfer from device complete.\n");
  rx_checksum = 0;
  for (i = 0; i < block_size / 4; i++)
    {
      rx_checksum += *((uint32_t *)test_block + i);
    }

  pciinfo("Received block checksum 0x%08x\n", rx_checksum);
  pciinfo("TEST %s\n", (rx_checksum == tx_checksum) ? "PASS" : "FAIL");
}

/*****************************************************************************
 * Name: qemu_edu_interrupt
 *
 * Description:
 *  EDU interrupt handler
 *
 *****************************************************************************/

static int qemu_edu_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct qemu_edu_priv_s *drv_priv = (struct qemu_edu_priv_s *)arg;
  uintptr_t                   base_addr;
  uint32_t                    status;

  base_addr = drv_priv->base_addr;
  status = qemu_edu_read_reg32(base_addr + EDU_REG_INT_STATUS);

  qemu_edu_write_reg32(base_addr + EDU_REG_INT_ACK, ~0U);
  switch (status)
    {
      /* Factorial triggered */

      case 0x1:
        {
          drv_priv->test_result
              = qemu_edu_read_reg32(base_addr + EDU_REG_FAC);
          pciinfo("Computed factorial: %d\n", drv_priv->test_result);
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
          drv_priv->test_result = status;
          pciinfo("Received value: 0x%08x\n", status);
          break;
        }
    }

  sem_post(&drv_priv->isr_done);
  return OK;
}

/*****************************************************************************
 * Name: qemu_edu_probe
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int qemu_edu_probe(FAR struct pci_bus_s *bus,
                          FAR const struct pci_dev_type_s *type,
                          uint16_t bdf)
{
  struct qemu_edu_priv_s drv_priv;
  struct pci_dev_s       dev;
  uint32_t               bar;
  uintptr_t              bar_addr;
  uint8_t                irq;

  /* Get dev */

  dev.bus = bus;
  dev.type = type;
  dev.bdf = bdf;

  pci_enable_bus_master(&dev);
  pciinfo("Enabled bus mastering\n");
  pci_enable_io(&dev, PCI_SYS_RES_MEM);
  pciinfo("Enabled memory resources\n");

  if (pci_bar_valid(&dev, EDU_CONTROL_BAR_ID) != OK)
    {
      pcierr("Control BAR is not valid\n");
      DEBUGPANIC();
      return -EINVAL;
    }

  bar_addr = pci_bar_addr(&dev, EDU_CONTROL_BAR_ID);
  bar = bus->ops->pci_cfg_read(&dev, EDU_CONTROL_BAR_OFFSET, 4);
  if ((bar & PCI_BAR_LAYOUT_MASK) != PCI_BAR_LAYOUT_MEM)
    {
      pcierr("Control bar expected to be MMIO\n");
      DEBUGPANIC();
      return -EINVAL;
    }

  if (bus->ops->pci_map_bar(bar_addr,
      pci_bar_size(&dev, EDU_CONTROL_BAR_ID)) != OK)
    {
      pcierr("Failed to map address space\n");
      DEBUGPANIC();
      return -EINVAL;
    }

  pciinfo("Device Initialized\n");

  /* Run Poll Tests */

  qemu_edu_test_poll(&dev, bar_addr);

  /* Run IRQ Tests */

  drv_priv.base_addr = bar_addr;
  sem_init(&drv_priv.isr_done, 0, 0);
  sem_setprotocol(&drv_priv.isr_done, SEM_PRIO_NONE);

  irq = IRQ0 + bus->ops->pci_cfg_read(&dev, PCI_HEADER_NORM_INT_LINE, 1);
  pciinfo("Attaching IRQ %d to %p\n", irq, qemu_edu_interrupt);
  irq_attach(irq, (xcpt_t)qemu_edu_interrupt, (void *)&drv_priv);
  up_enable_irq(irq);

  qemu_edu_test_intx(&dev, &drv_priv);
  qemu_edu_test_dma(&dev, &drv_priv);

  up_disable_irq(irq);
  irq_detach(irq);
  sem_destroy(&drv_priv.isr_done);

  /* Run MSI Tests */

  /* Really should be cleaning up the mapped memory */

  return OK;
}
