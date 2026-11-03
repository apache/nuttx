======================================
USB Video Class (UVC) Host Driver
======================================

Overview
========

The UVC host driver (``drivers/usbhost/usbhost_uvc.c``) enumerates USB
Video Class 1.1/1.5 cameras attached to an isochronous capable host
controller and exposes each camera through the NuttX camera upper half as
``/dev/videoN``.  A camera can then be used with any application that
speaks the V4L2 capture API, ``apps/examples/camera`` for instance.

The driver parses the VideoControl and VideoStreaming descriptor sets of
the camera, answers the class-specific control requests (including
PROBE/COMMIT negotiation) and streams frames from the isochronous IN
endpoint that the negotiated alternate setting provides.

Features
========

- UVC 1.1/1.5 cameras, both composite (IAD) and single function devices
- MJPEG and frame-based (H.264/HEVC) payloads
- Uncompressed payloads: YUYV, UYVY, NV12, YVU420, YUV420, M420, GREY,
  RGB565 and BGR24, matched by the format GUID
- Format, frame size and frame interval enumeration
- Alternate setting selection driven by the bandwidth the frame size needs
- A pipeline of isochronous transfers, sized in packets, so a frame may
  span several transfers without losing microframe data
- Stream restart on transport errors and safe teardown while transfers
  are still pending

Configuration
=============

The driver is enabled through the following Kconfig options:

.. code-block:: kconfig

   CONFIG_VIDEO_STREAM=y              # Required: camera upper half
   CONFIG_USBHOST_HAVE_ASYNCH=y       # Required: provided by the host controller
   CONFIG_SCHED_HPWORK=y              # Required: disconnect handling
   CONFIG_USBHOST_COMPOSITE=y         # Recommended: UVC cameras are usually composite

   CONFIG_USBHOST_UVC=y               # Enable the driver
   CONFIG_USBHOST_UVC_ISO_PIPELINE_DEPTH=2
   CONFIG_USBHOST_UVC_PACKETS_PER_SLOT=32
   CONFIG_USBHOST_UVC_THREAD_PRIORITY=100
   CONFIG_USBHOST_UVC_THREAD_STACKSIZE=4096

``CONFIG_USBHOST_UVC_ISO_PIPELINE_DEPTH`` is the number of isochronous
transfers kept in flight; ``CONFIG_USBHOST_UVC_PACKETS_PER_SLOT`` is how
many packets each of them carries.  A slot is not a frame: frame
boundaries come from the UVC payload header, so a large frame spans as
many slots as it needs.  Each slot costs

.. code-block:: text

   PACKETS_PER_SLOT * wMaxPacketSize        (heap, per slot)
   ISO_PIPELINE_DEPTH * the above           (total)

``PACKETS_PER_SLOT`` must be at least the number of packets the host
controller delivers per completion callback, otherwise packets are
dropped whenever the pending queue runs shallow.

Device Nodes
============

Every enumerated camera registers itself as ``/dev/video0``,
``/dev/video1`` and so on, up to ``UVC_MAX_DEVICES`` cameras.  The node
implements the V4L2 capture ioctls through the camera upper half:

- ``VIDIOC_QUERYCAP``, ``VIDIOC_ENUM_FMT``, ``VIDIOC_S_FMT`` / ``G_FMT``
- ``VIDIOC_ENUM_FRAMESIZES``, ``VIDIOC_ENUM_FRAMEINTERVALS``
- ``VIDIOC_REQBUFS``, ``VIDIOC_QBUF``, ``VIDIOC_DQBUF``, ``VIDIOC_STREAMON``
  / ``VIDIOC_STREAMOFF``
- ``VIDIOC_QUERYCTRL``, ``VIDIOC_G_CTRL``, ``VIDIOC_S_CTRL``

Streaming Model
===============

For a given frame size the driver picks the alternate setting whose
``wMaxPacketSize`` covers the bandwidth the format needs, or falls back
to the largest one available.  It then programs the camera through
``VS_PROBE_CONTROL`` / ``VS_COMMIT_CONTROL`` and starts the isochronous
pipeline.

Payload headers are parsed as the transfers complete: the frame
identifier tells the driver when a new frame starts, and the end of
frame bit terminates it.  Completed frames are handed to the upper half
through the ``imgdata`` capture callback, which copies them into the
buffer the application queued.

Testing on the Simulator
========================

The driver can be exercised on the simulator with a real camera passed
through to the host by libusb:

.. code-block:: kconfig

   CONFIG_SIM_USB_HOST=y
   CONFIG_SIM_LIBUSB=y
   CONFIG_SIM_USB_VID=0xXXXX           # Camera vendor ID
   CONFIG_SIM_USB_PID=0xXXXX           # Camera product ID
   CONFIG_VIDEO_STREAM=y
   CONFIG_USBHOST_UVC=y

Enable the camera example in ``apps``, boot the simulator and capture:

.. code-block:: text

   nsh> camera

The host must allow the simulator process to claim the camera interface,
which usually means the process runs as ``root`` or a udev rule grants
access to the device.

Limitations
===========

- One streaming interface per camera; a camera that exposes several video
  streaming interfaces registers only the first one.
- Isochronous transfers only.  Cameras that use bulk endpoints for video
  data are not supported.
- High bandwidth alternate settings (multiple transactions per microframe)
  depend on the host controller programming ``Max Burst Size`` correctly; a
  controller that hard codes a burst size of zero will run the camera at a
  fraction of the negotiated bandwidth.
- Uncompressed payloads are passed through as-is; no colour space
  conversion is performed by the driver.
