/****************************************************************************
 * drivers/virtio/virtio-pci.c
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
#include <stdint.h>
#include <sys/param.h>

#include <nuttx/virtio/virtio-pci.h>

#include "virtio-pci.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int virtio_pci_probe(FAR struct pci_device_s *dev);
static void virtio_pci_remove(FAR struct pci_device_s *dev);

static int virtio_pci_create_virtqueue(FAR struct virtio_pci_device_s *vpdev,
                                       unsigned int i, FAR const char *name,
                                       vq_callback callback);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pci_device_id_s g_virtio_pci_id_table[] =
{
  { PCI_DEVICE(PCI_VENDOR_ID_REDHAT_QUMRANET, PCI_ANY_ID) },
  { 0, }
};

static struct pci_driver_s g_virtio_pci_drv =
{
  g_virtio_pci_id_table,
  virtio_pci_probe,
  virtio_pci_remove
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 * Name: virtio_pci_probe
 ****************************************************************************/

static int virtio_pci_probe(FAR struct pci_device_s *dev)
{
  FAR struct virtio_pci_device_s *vpdev;
  FAR struct virtio_device *vdev;
  int ret;

  /* We only own devices >= 0x1000 and <= 0x107f: leave the rest. */

  if (dev->device < 0x1000 || dev->device > 0x107f)
    {
      pcierr("Pci device id err, id=%d\n", dev->device);
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

  ret = pci_enable_device(dev);
  if (ret < 0)
    {
      vrterr("Enable virtio pci device failed, ret=%d\n", ret);
      goto err;
    }

  pci_set_master(dev);
  ret = virtio_pci_modern_probe(dev);
  if (ret == -ENODEV)
    {
      ret = virtio_pci_legacy_probe(dev);
      if (ret < 0)
        {
          vrterr("Virtio pci legacy probe failed\n");
          goto err_with_enable;
        }
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

/****************************************************************************
 * Name: virtio_pci_remove
 ****************************************************************************/

static void virtio_pci_remove(FAR struct pci_device_s *dev)
{
  FAR struct virtio_pci_device_s *vpdev = dev->priv;

  virtio_unregister_device(&vpdev->vdev);

  irq_detach(vpdev->irq[VIRTIO_PCI_INT_CFG]);
  irq_detach(vpdev->irq[VIRTIO_PCI_INT_VQ]);
  pci_release_irq(vpdev->dev, vpdev->irq, VIRTIO_PCI_INT_NUM);

  pci_clear_master(dev);
  pci_disable_device(dev);
  kmm_free(vpdev);
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
  vralloc->num_descs = vpdev->ops->get_queue_len(vpdev, i);
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

  /* Set the pci virtqueue register, active vq, enable vq */

  ret = vpdev->ops->create_virtqueue(vpdev, vq);
  if (ret < 0)
    {
      vrterr("Virtio_pci_config_virtqueue failed, ret=%d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_pci_create_virtqueues
 ****************************************************************************/

int virtio_pci_create_virtqueues(FAR struct virtio_device *vdev,
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

  ret = vpdev->ops->config_vector(vpdev);
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

void virtio_pci_delete_virtqueues(FAR struct virtio_device *vdev)
{
  FAR struct virtio_pci_device_s *vpdev =
    (FAR struct virtio_pci_device_s *)vdev;
  FAR struct virtio_vring_info *vrinfo;
  unsigned int i;

  /* Disable interrupt first */

  up_disable_irq(vpdev->irq[VIRTIO_PCI_INT_CFG]);
  up_disable_irq(vpdev->irq[VIRTIO_PCI_INT_VQ]);

  /* Free the memory */

  if (vdev->vrings_info != NULL)
    {
      for (i = 0; i < vdev->vrings_num; i++)
        {
          vrinfo = &vdev->vrings_info[i];
          vpdev->ops->delete_virtqueue(vdev, i);

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
 * Name: virtio_pci_negotiate_features
 ****************************************************************************/

uint32_t
virtio_pci_negotiate_features(FAR struct virtio_device *vdev,
                                     uint32_t features)
{
  features = features & vdev->func->get_features(vdev);
  vdev->func->set_features(vdev, features);
  return features;
}

/****************************************************************************
 * Name: virtio_pci_reset_device
 ****************************************************************************/

void virtio_pci_reset_device(FAR struct virtio_device *vdev)
{
  vdev->func->set_features(vdev, VIRTIO_CONFIG_STATUS_RESET);
}

/****************************************************************************
 * Name: register_virtio_pci_driver
 ****************************************************************************/

int register_virtio_pci_driver(void)
{
  return pci_register_driver(&g_virtio_pci_drv);
}
