=============================================
``uvc_cam`` UVC Camera streaming application
=============================================

Captures frames from a V4L2 camera sensor and streams them to a USB host
via the UVC (USB Video Class) gadget driver (``/dev/uvc0``).

The application queries the sensor's native pixel format, resolution and
frame rate via V4L2, then continuously captures frames and writes them to
the UVC device node.

Dependencies
==========================

- :code:`CONFIG_USBUVC=y` – UVC gadget driver
- :code:`CONFIG_VIDEO=y` – Video subsystem
- :code:`CONFIG_BOARDCTL_USBDEVCTRL=y` – USB device control via boardctl

Configuration
==========================

- :code:`CONFIG_EXAMPLES_UVC_CAM=y` – Enable the UVC camera example
- :code:`CONFIG_EXAMPLES_UVC_CAM_NFRAMES` – Number of frames to stream (0 = infinite, default)
- :code:`CONFIG_EXAMPLES_UVC_CAM_PRIORITY` – Task priority (default 100)
- :code:`CONFIG_EXAMPLES_UVC_CAM_STACKSIZE` – Stack size (default 4096)

Supported Pixel Formats
==========================

- ``YUYV`` (2 bytes per pixel)
- ``RGB565`` (2 bytes per pixel)
- ``RGB24`` (3 bytes per pixel)

Usage
==========================

.. code-block:: bash

   uvc_cam [nframes] [video_dev] [uvc_dev]

- ``nframes`` – Number of frames to capture (default from Kconfig, 0 = infinite)
- ``video_dev`` – V4L2 camera device path (default ``/dev/video0``)
- ``uvc_dev`` – UVC gadget device path (default ``/dev/uvc0``)

Examples
==========================

- Stream indefinitely with defaults: :code:`uvc_cam`
- Stream 100 frames: :code:`uvc_cam 100`
- Use a different camera device: :code:`uvc_cam 0 /dev/video1`
- Specify both camera and UVC device: :code:`uvc_cam 0 /dev/video0 /dev/uvc1`

Board Support
==========================

The ``lckfb-szpi-esp32s3`` board provides a ``uvc`` configuration with
camera and UVC gadget driver pre-enabled::

   $ ./tools/configure.sh lckfb-szpi-esp32s3:uvc

This configuration enables the ESP32-S3 camera interface (GC0308 sensor),
USB OTG and the UVC gadget driver.  The ``uvc_cam`` example application is
not enabled by default and must be turned on manually::

   $ kconfig-tweak --enable CONFIG_EXAMPLES_UVC_CAM

Or via ``make menuconfig``:
:menuselection:`Application Configuration --> Examples --> UVC Camera streaming example`

After flashing, run the application manually from the NSH shell::

   nsh> uvc_cam
