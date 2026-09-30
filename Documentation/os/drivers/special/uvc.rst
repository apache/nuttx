======================================
USB Video Class (UVC) Gadget Driver
======================================

Overview
========

The UVC gadget driver (``drivers/usbdev/uvc.c``) implements a USB Video Class
1.1 device that makes NuttX appear as a USB webcam to the host.  The driver
exposes a character device (``/dev/uvc0``) that applications write video frames
to.  It handles all UVC class-specific control requests (PROBE / COMMIT)
internally and uses bulk transfers for video data.

The driver supports two modes:

- **Standalone** – the UVC function is the only USB class on the bus.
- **Composite** – the UVC function is combined with other USB class drivers
  (e.g. CDC/ACM) via the NuttX composite device framework.

Features
========

- UVC 1.1 compliant (uncompressed YUY2 format)
- Bulk transfer mode for video data
- Automatic PROBE / COMMIT negotiation with the host
- ``poll()`` support – applications can wait for the host to start streaming
  (``POLLOUT``) and detect disconnection (``POLLHUP``)
- Runtime video parameters – resolution and frame rate are passed at
  initialization time, so USB descriptors always match the actual sensor
- ``boardctl()`` integration for easy application-level bring-up

Configuration
=============

The driver is enabled through the following Kconfig options:

.. code-block:: kconfig

   CONFIG_USBUVC=y                  # Enable UVC gadget support
   CONFIG_USBUVC_COMPOSITE=n        # Set y for composite device mode
   CONFIG_USBUVC_EPBULKIN=1         # Bulk IN endpoint number (standalone)
   CONFIG_USBUVC_EP0MAXPACKET=64    # EP0 max packet size (standalone)
   CONFIG_USBUVC_EPBULKIN_FSSIZE=64 # Bulk IN full-speed max packet size
   CONFIG_USBUVC_NWRREQS=4          # Number of pre-allocated write requests
   CONFIG_USBUVC_NPOLLWAITERS=2     # Number of poll waiters

Header Files
============

- ``include/nuttx/usb/uvc.h`` – Public API, UVC descriptor constants, and
  data structures.

Data Structures
===============

``struct uvc_params_s``
-----------------------

Passed to the initialization function so that USB descriptors reflect the
actual sensor capabilities::

  struct uvc_params_s
  {
    uint16_t width;    /* Frame width in pixels  */
    uint16_t height;   /* Frame height in pixels */
    uint8_t  fps;      /* Frames per second      */
  };

Public Interfaces
=================

Standalone Mode
---------------

``usbdev_uvc_initialize()``
  Initialize the UVC gadget and register ``/dev/uvc0``.  *params* may be
  ``NULL`` to use defaults (320 × 240 @ 5 fps).  Returns a handle for later
  ``usbdev_uvc_uninitialize()``.

``usbdev_uvc_uninitialize()``
  Tear down the UVC gadget and unregister the character device.

Composite Mode
--------------

``usbdev_uvc_classobject()``
  Create a UVC class driver instance for use inside a composite device.

``usbdev_uvc_classuninitialize()``
  Destroy a class driver instance created by ``usbdev_uvc_classobject()``.

``usbdev_uvc_get_composite_devdesc()``
  Fill a ``composite_devdesc_s`` for the composite framework.

boardctl Integration
--------------------

Applications can manage the UVC gadget through ``boardctl()``::

  struct boardioc_usbdev_ctrl_s ctrl;
  struct uvc_params_s params = { .width = 320, .height = 240, .fps = 15 };
  FAR void *handle = (FAR void *)&params;

  ctrl.usbdev   = BOARDIOC_USBDEV_UVC;
  ctrl.action   = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = 0;
  ctrl.config   = 0;
  ctrl.handle   = &handle;

  boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);

  /* ... use /dev/uvc0 ... */

  ctrl.action = BOARDIOC_USBDEV_DISCONNECT;
  boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);

Device Operation
================

1. The application opens ``/dev/uvc0`` for writing.
2. Use ``poll()`` with ``POLLOUT`` to wait for the USB host to start streaming
   (i.e. the host sends a VS_COMMIT_CONTROL SET_CUR request).
3. Write complete video frames via ``write()``.  The driver prepends a 2-byte
   UVC payload header (with FID / EOF bits) automatically.
4. When the host stops streaming, ``write()`` returns ``-EAGAIN``.  The
   application can ``poll()`` again to wait for the host to restart.
5. ``POLLHUP`` is reported when the host explicitly stops streaming.
