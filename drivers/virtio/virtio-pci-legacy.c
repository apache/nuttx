/****************************************************************************
 * drivers/virtio/virtio-pci-legacy.c
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

#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <sys/param.h>

#include "virtio-pci.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_PCI_LEGACY_IO_BAR            0

/* A 32-bit r/o bitmask of the features supported by the host */

#define VIRTIO_PCI_HOST_FEATURES            0

/* A 32-bit r/w bitmask of features activated by the guest */

#define VIRTIO_PCI_GUEST_FEATURES           4

/* A 32-bit r/w PFN for the currently selected queue */

#define VIRTIO_PCI_QUEUE_PFN                8

/* A 16-bit r/o queue size for the currently selected queue */

#define VIRTIO_PCI_QUEUE_NUM                12

/* A 16-bit r/w queue selector */

#define VIRTIO_PCI_QUEUE_SEL                14

/* A 16-bit r/w queue notifier */

#define VIRTIO_PCI_QUEUE_NOTIFY             16

/* An 8-bit device status register.  */

#define VIRTIO_PCI_STATUS                   18

/* An 8-bit r/o interrupt status register.  Reading the value will return the
 * current contents of the ISR and will also clear it.  This is effectively
 * a read-and-acknowledge.
 */

#define VIRTIO_PCI_ISR                      19

/* MSI-X registers: only enabled if MSI-X is enabled. */

#define VIRTIO_MSI_CONFIG_VECTOR            20

/* A 16-bit vector for selected queue notifications. */

#define VIRTIO_MSI_QUEUE_VECTOR             22

/* The remaining space is defined by each driver as the per-driver
 * configuration space
 */

#define VIRTIO_PCI_CONFIG_OFF(msix_enabled) ((msix_enabled) ? 24 : 20)

/* Virtio ABI version, this must match exactly */

#define VIRTIO_PCI_ABI_VERSION              0

/* How many bits to shift physical queue address written to QUEUE_PFN.
 * 12 is historical, and due to x86 page size.
 */

#define VIRTIO_PCI_QUEUE_ADDR_SHIFT         12

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static uint16_t
virtio_pci_get_queue_len(FAR struct virtio_pci_device_s *vpdev, int idx);
static int virtio_pci_config_virtqueue(FAR struct virtio_pci_device_s *vpdev,
                                       FAR struct virtqueue *vq);
static int virtio_pci_init_device(FAR struct virtio_pci_device_s *vpdev);
static int virtio_pci_create_virtqueue(FAR struct virtio_pci_device_s *vpdev,
                                       unsigned int i, FAR const char *name,
                                       vq_callback callback);

/* Virtio pci dispatch functions */

static int virtio_pci_create_virtqueues(FAR struct virtio_device *vdev,
                                        unsigned int flags,
                                        unsigned int nvqs,
                                        FAR const char *names[],
                                        vq_callback callbacks[]);
static void virtio_pci_delete_virtqueues(FAR struct virtio_device *vdev);
static void virtio_pci_set_status(FAR struct virtio_device *vdev,
                                  uint8_t status);
static uint8_t virtio_pci_get_status(FAR struct virtio_device *vdev);
static void virtio_pci_write_config(FAR struct virtio_device *vdev,
                                    uint32_t offset, FAR void *dst,
                                    int length);
static void virtio_pci_read_config(FAR struct virtio_device *vdev,
                                   uint32_t offset, FAR void *dst,
                                   int length);
static uint32_t virtio_pci_get_features(FAR struct virtio_device *vdev);
static void virtio_pci_set_features(FAR struct virtio_device *vdev,
                                    uint32_t features);
static uint32_t virtio_pci_negotiate_features(FAR struct virtio_device *vdev,
                                              uint32_t features);
static void virtio_pci_reset_device(FAR struct virtio_device *vdev);
static void virtio_pci_notify(FAR struct virtqueue *vq);

/* Interrupt */

static int virtio_pci_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct virtio_dispatch g_virtio_pci_dispatch =
{
  virtio_pci_create_virtqueues,  /* create_virtqueues */
  virtio_pci_delete_virtqueues,  /* delete_virtqueues */
  virtio_pci_get_status,         /* get_status */
  virtio_pci_set_status,         /* set_status */
  virtio_pci_get_features,       /* get_features */
  virtio_pci_set_features,       /* set_features */
  virtio_pci_negotiate_features, /* negotiate_features */
  virtio_pci_read_config,        /* read_config */
  virtio_pci_write_config,       /* write_config */
  virtio_pci_reset_device,       /* reset_device */
  virtio_pci_notify,             /* notify */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_pci_get_queue_len
 ****************************************************************************/

static uint16_t
virtio_pci_get_queue_len(FAR struct virtio_pci_device_s *vpdev, int idx)
{
  uint16_t num;

  pci_write_io_word(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_QUEUE_SEL, idx);
  pci_read_io_word(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_QUEUE_NUM, &num);
  if (num == 0)
    {
      pcierr("Queue is not available num=%d\n", num);
    }

  return num;
}

/****************************************************************************
 * Name: virtio_pci_config_virtqueue
 ****************************************************************************/

static int virtio_pci_config_virtqueue(FAR struct virtio_pci_device_s *vpdev,
                                       FAR struct virtqueue *vq)
{
  uint16_t msix_vector;

  /* Set the pci virtqueue register, active vq, enable vq */

  pci_write_io_word(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_QUEUE_SEL,
                    vq->vq_queue_index);

  /* activate the queue */

  pci_write_io_dword(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_QUEUE_PFN,
                     up_addrenv_va_to_pa(vq->vq_ring.desc) >>
                     VIRTIO_PCI_QUEUE_ADDR_SHIFT);
  pci_write_io_word(vpdev->dev, vpdev->ioaddr + VIRTIO_MSI_QUEUE_VECTOR,
                    VIRTIO_PCI_INT_VQ);
  pci_read_io_word(vpdev->dev, vpdev->ioaddr + VIRTIO_MSI_QUEUE_VECTOR,
                   &msix_vector);
  if (msix_vector == VIRTIO_PCI_MSI_NO_VECTOR)
    {
      pci_write_io_dword(vpdev->dev,
                         vpdev->ioaddr + VIRTIO_PCI_QUEUE_PFN, 0);
      vrterr("Msix vector is 0\n");
      return -EBUSY;
    }

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_config_vector
 ****************************************************************************/

static int
virtio_pci_config_vector(FAR struct virtio_pci_device_s *vpdev)
{
  uint16_t rvector;

  pci_write_io_word(vpdev->dev, vpdev->ioaddr + VIRTIO_MSI_CONFIG_VECTOR,
                    VIRTIO_PCI_INT_CFG);
  pci_read_io_word(vpdev->dev, vpdev->ioaddr + VIRTIO_MSI_CONFIG_VECTOR,
                   &rvector);
  if (rvector == VIRTIO_PCI_MSI_NO_VECTOR)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_create_virtqueue
 ****************************************************************************/

static int
virtio_pci_create_virtqueue(FAR struct virtio_pci_device_s *vpdev,
                            unsigned int i, FAR const char *name,
                            vq_callback callback)
{
  FAR struct virtio_device *vdev = &vpdev->vdev;
  FAR struct virtio_vring_info *vrinfo;
  FAR struct vring_alloc_info *vralloc;
  FAR struct virtqueue *vq;
  int vringsize;
  int ret;

  /* Alloc virtqueue and init the vring info and vring alloc info */

  vrinfo  = &vdev->vrings_info[i];
  vralloc = &vrinfo->info;
  vralloc->num_descs = virtio_pci_get_queue_len(vpdev, i);
  vq = virtqueue_allocate(vralloc->num_descs);
  if (vq == NULL)
    {
      vrterr("Virtqueue_allocate failed\n");
      return -ENOMEM;
    }

  /* Init the vring info and vring alloc info */

  vrinfo->vq = vq;
  vrinfo->io = &vpdev->shm_io;
  vralloc->align = VIRTIO_PCI_VRING_ALIGN;
  vringsize = vring_size(vralloc->num_descs, VIRTIO_PCI_VRING_ALIGN);
  vralloc->vaddr = virtio_zalloc_buf(vdev, vringsize,
                                     VIRTIO_PCI_VRING_ALIGN);
  if (vralloc->vaddr == NULL)
    {
      vrterr("Vring alloc failed\n");
      return -ENOMEM;
    }

  /* Initialize the virtio queue */

  ret = virtqueue_create(vdev, i, name, vralloc, callback,
                         vdev->func->notify, vq);
  if (ret < 0)
    {
      vrterr("Virtqueue create error, ret=%d\n", ret);
      return ret;
    }

  virtqueue_set_shmem_io(vq, &vpdev->shm_io);

  ret = virtio_pci_config_virtqueue(vpdev, vq);
  if (ret < 0)
    {
      vrterr("Virtio_pci_config_virtqueue failed, ret=%d\n", ret);
      goto err;
    }

  return OK;

err:
  pci_write_io_dword(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_QUEUE_PFN, 0);
  return ret;
}

/****************************************************************************
 * Name: virtio_pci_create_virtqueues
 ****************************************************************************/

static int virtio_pci_create_virtqueues(FAR struct virtio_device *vdev,
                                        unsigned int flags,
                                        unsigned int nvqs,
                                        FAR const char *names[],
                                        vq_callback callbacks[])
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  unsigned int i;
  int ret = OK;

  /* Alloc vring info */

  vdev->vrings_num = nvqs;
  vdev->vrings_info = kmm_zalloc(sizeof(struct virtio_vring_info) * nvqs);
  if (vdev->vrings_info == NULL)
    {
      vrterr("alloc vrings info failed\n");
      return -ENOMEM;
    }

  ret = virtio_pci_config_vector(vpdev);
  if (ret < 0)
    {
      vrterr("read virtio pci config msix vector failed\n");
      return ret;
    }

  /* Alloc and init the virtqueue */

  for (i = 0; i < nvqs; i++)
    {
      ret = virtio_pci_create_virtqueue(vpdev, i, names[i], callbacks[i]);
      if (ret < 0)
        {
          goto err;
        }
    }

  /* Finally, enable the interrupt */

  up_enable_irq(vpdev->irq[VIRTIO_PCI_INT_CFG]);
  up_enable_irq(vpdev->irq[VIRTIO_PCI_INT_VQ]);
  return ret;

err:
  virtio_pci_delete_virtqueues(vdev);
  return ret;
}

/****************************************************************************
 * Name: virtio_pci_delete_virtqueues
 ****************************************************************************/

static void virtio_pci_delete_virtqueues(FAR struct virtio_device *vdev)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  FAR struct virtio_vring_info *vrinfo;
  unsigned int i;
  uint8_t isr;

  /* Disable interrupt first */

  up_disable_irq(vpdev->irq[VIRTIO_PCI_INT_CFG]);
  up_disable_irq(vpdev->irq[VIRTIO_PCI_INT_VQ]);

  /* Free the memory */

  if (vdev->vrings_info != NULL)
    {
      for (i = 0; i < vdev->vrings_num; i++)
        {
          vrinfo = &vdev->vrings_info[i];

          pci_write_io_word(vpdev->dev,
                            vpdev->ioaddr + VIRTIO_PCI_QUEUE_SEL, i);
          pci_write_io_word(vpdev->dev,
                            vpdev->ioaddr + VIRTIO_MSI_QUEUE_VECTOR,
                            VIRTIO_PCI_MSI_NO_VECTOR);

          /* Flush the write out to device */

          pci_read_io_byte(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_ISR, &isr);

          /* Select and deactivate the queue */

          pci_write_io_dword(vpdev->dev,
                             vpdev->ioaddr + VIRTIO_PCI_QUEUE_PFN, 0);

          /* Free the vring buffer and virtqueue */

          if (vrinfo->info.vaddr != NULL)
            {
              virtio_free_buf(vdev, vrinfo->info.vaddr);
            }

          if (vrinfo->vq != NULL)
            {
              virtqueue_free(vrinfo->vq);
            }
        }

      kmm_free(vdev->vrings_info);
    }
}

/****************************************************************************
 * Name: virtio_pci_set_status
 ****************************************************************************/

static void virtio_pci_set_status(FAR struct virtio_device *vdev,
                                  uint8_t status)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;

  pci_write_io_byte(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_STATUS, status);
}

/****************************************************************************
 * Name: virtio_pci_get_status
 ****************************************************************************/

static uint8_t virtio_pci_get_status(FAR struct virtio_device *vdev)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  uint8_t status;

  pci_read_io_byte(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_STATUS, &status);
  return status;
}

/****************************************************************************
 * Name: virtio_pci_write_config
 ****************************************************************************/

static void virtio_pci_write_config(FAR struct virtio_device *vdev,
                                    uint32_t offset, FAR void *src,
                                    int length)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  FAR uint8_t *s = src;
  int i;

  for (i = 0; i < length; i++)
    {
      pci_write_io_byte(vpdev->dev, vpdev->ioaddr +
                        VIRTIO_PCI_CONFIG_OFF(true) + offset + i, s[i]);
    }
}

/****************************************************************************
 * Name: virtio_pci_read_config
 ****************************************************************************/

static void virtio_pci_read_config(FAR struct virtio_device *vdev,
                                   uint32_t offset, FAR void *dst,
                                   int length)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  FAR uint8_t *d = dst;
  int i;

  for (i = 0; i < length; i++)
    {
      pci_read_io_byte(vpdev->dev, vpdev->ioaddr +
                       VIRTIO_PCI_CONFIG_OFF(true) + offset + i, &d[i]);
    }
}

/****************************************************************************
 * Name: virtio_pci_get_features
 ****************************************************************************/

static uint32_t virtio_pci_get_features(FAR struct virtio_device *vdev)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  uint32_t feature;

  pci_read_io_dword(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_HOST_FEATURES,
                    &feature);
  return feature;
}

/****************************************************************************
 * Name: virtio_pci_set_features
 ****************************************************************************/

static void virtio_pci_set_features(FAR struct virtio_device *vdev,
                                    uint32_t features)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;

  pci_write_io_dword(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_GUEST_FEATURES,
                     vdev->features);
}

/****************************************************************************
 * Name: virtio_pci_negotiate_features
 ****************************************************************************/

static uint32_t virtio_pci_negotiate_features(FAR struct virtio_device *vdev,
                                              uint32_t features)
{
  features = features & virtio_pci_get_features(vdev);
  virtio_pci_set_features(vdev, features);
  return features;
}

/****************************************************************************
 * Name: virtio_pci_reset_device
 ****************************************************************************/

static void virtio_pci_reset_device(FAR struct virtio_device *vdev)
{
  virtio_pci_set_status(vdev, VIRTIO_CONFIG_STATUS_RESET);
}

/****************************************************************************
 * Name: virtio_pci_notify
 ****************************************************************************/

static void virtio_pci_notify(FAR struct virtqueue *vq)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vq->vq_dev;

  pci_write_io_word(vpdev->dev, vpdev->ioaddr + VIRTIO_PCI_QUEUE_NOTIFY,
                    vq->vq_queue_index);
}

/****************************************************************************
 * Name: virtio_pci_config_changed
 ****************************************************************************/

static int virtio_pci_config_changed(int irq, FAR void *context,
                                     FAR void *arg)
{
  /* TODO: not support config changed notification */

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_interrupt
 ****************************************************************************/

static int virtio_pci_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct virtio_pci_device_s *vpdev = arg;
  FAR struct virtio_vring_info *vrings_info = vpdev->vdev.vrings_info;
  FAR struct virtqueue *vq;
  unsigned int i;

  for (i = 0; i < vpdev->vdev.vrings_num; i++)
    {
      vq = vrings_info[i].vq;
      if (vq->vq_used_cons_idx != vq->vq_ring.used->idx &&
          vq->callback != NULL)
        {
          vq->callback(vq);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_init_device
 ****************************************************************************/

static int virtio_pci_init_device(FAR struct virtio_pci_device_s *vpdev)
{
  FAR struct virtio_device *vdev = &vpdev->vdev;
  FAR struct pci_device_s *dev = vpdev->dev;

  if (dev->revision != VIRTIO_PCI_ABI_VERSION)
    {
      pcierr("Virtio_pci: expected ABI version %d, got %u\n",
              VIRTIO_PCI_ABI_VERSION, dev->revision);
      return -ENODEV;
    }

  vpdev->ioaddr = pci_map_bar(dev, VIRTIO_PCI_LEGACY_IO_BAR);
  if (vpdev->ioaddr == NULL)
    {
      vrterr("Unable to map virtio on bar\n");
      return -EINVAL;
    }

  vdev->id.vendor = dev->subsystem_vendor;
  vdev->id.device = dev->subsystem_device;

  return OK;
}

/****************************************************************************
 * Name: virtio_pci_legacy_probe
 ****************************************************************************/

int virtio_pci_legacy_probe(FAR struct pci_device_s *dev)
{
  FAR struct virtio_pci_device_s *vpdev;
  FAR struct virtio_device *vdev;
  int ret;

  /* We only own devices >= 0x1000 and <= 0x107f: leave the rest. */

  if (dev->device < 0x1000 || dev->device > 0x103f)
    {
      return -ENODEV;
    }

  vpdev = kmm_zalloc(sizeof(*vpdev));
  if (vpdev == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  dev->priv = vpdev;
  vpdev->dev = dev;
  vdev = &vpdev->vdev;

  /* Virtio device initialize */

  metal_io_init(&vpdev->shm_io, NULL, &vpdev->shm_phy,
                SIZE_MAX, UINT_MAX, 0, metal_io_get_ops());

  vdev->func = &g_virtio_pci_dispatch;
  vdev->role = VIRTIO_DEV_DRIVER;

  ret = pci_enable_device(dev);
  if (ret < 0)
    {
      vrterr("Enable virtio pci device failed, ret=%d\n", ret);
      goto err;
    }

  pci_set_master(dev);

  ret = virtio_pci_init_device(vpdev);
  if (ret < 0)
    {
      vrterr("Virtio pci legacy device init failed, ret=%d\n", ret);
      goto err_with_enable;
    }

  /* Irq init */

  ret = pci_alloc_irq(vpdev->dev, vpdev->irq, VIRTIO_PCI_INT_NUM);
  if (ret != VIRTIO_PCI_INT_NUM)
    {
      vrterr("Failed to allocate MSI %d\n", ret);
      goto err_with_enable;
    }

  vrtinfo("Attaching MSI %d to %p,  %d to %p\n",
          vpdev->irq[VIRTIO_PCI_INT_CFG], virtio_pci_config_changed,
          vpdev->irq[VIRTIO_PCI_INT_VQ], virtio_pci_interrupt);

  ret = pci_connect_irq(vpdev->dev, vpdev->irq, VIRTIO_PCI_INT_NUM);
  if (ret < 0)
    {
      vrterr("Failed to connect MSI %d\n", ret);
      goto err_with_irq;
    }

  irq_attach(vpdev->irq[VIRTIO_PCI_INT_CFG],
             virtio_pci_config_changed, vpdev);
  irq_attach(vpdev->irq[VIRTIO_PCI_INT_VQ], virtio_pci_interrupt, vpdev);

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_RESET);
  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_ACK);

  ret = virtio_register_device(&vpdev->vdev);
  if (ret < 0)
    {
      vrterr("Register virtio device failed, ret=%d\n", ret);
      goto err_with_attach;
    }

  return ret;

err_with_attach:
  irq_detach(vpdev->irq[VIRTIO_PCI_INT_CFG]);
  irq_detach(vpdev->irq[VIRTIO_PCI_INT_VQ]);
err_with_irq:
  pci_release_irq(vpdev->dev, vpdev->irq, VIRTIO_PCI_INT_NUM);
err_with_enable:
  pci_clear_master(dev);
  pci_disable_device(dev);
err:
  kmm_free(vpdev);
  return ret;
}
