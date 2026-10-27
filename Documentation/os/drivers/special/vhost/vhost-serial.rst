============
Vhost Serial
============

The vhost serial driver is the device side of virtio-console. It forwards
the data between the peer's virtio-console driver and a local character
device: data the peer queues on the TX virtqueue is written to the local
device, and data read from the local device is returned on the RX
virtqueue. It handles both ``VIRTIO_ID_CONSOLE`` and
``VIRTIO_ID_RPROC_SERIAL``.

Configuration
=============

- ``CONFIG_DRIVERS_VHOST_SERIAL``: enable the driver. It selects
  ``CONFIG_DRIVERS_VHOST``.
- ``CONFIG_DRIVERS_VHOST_SERIAL_PRIORITY``: priority of the RX and TX
  kernel threads (default 224).
- ``CONFIG_DRIVERS_VHOST_SERIAL_STACKSIZE``: stack size of the RX and TX
  kernel threads (default ``CONFIG_DEFAULT_TASK_STACKSIZE``).

Device config space
===================

The device config space holds ``struct vhost_serial_config_s`` from
``include/nuttx/virtio/virtio-serial.h``:

.. code-block:: c

   begin_packed_struct struct vhost_serial_config_s
   {
     struct virtio_serial_config_s config;
     char path[VIRTIO_SERIAL_PATH_SIZE];
   } end_packed_struct;

``path`` names the local character device the driver opens at probe time,
for example ``"/dev/ttyS1"``. The resource table author fills it in; the
driver treats the last byte as the string terminator, so a path must be
shorter than ``VIRTIO_SERIAL_PATH_SIZE`` (64) bytes. Probe fails if the
device cannot be opened.
