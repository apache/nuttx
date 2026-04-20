=============================================
``nxcamera`` Camera/Video Stream Test Command
=============================================

Introduction
============

``nxcamera`` is a command-line utility for testing camera devices and video
stream capture in NuttX. It is built on top of the NuttX video subsystem
(using a V4L2-style interface) and is typically used to:

- Enumerate and open video device nodes such as ``/dev/video0`` and
  ``/dev/video1``
- Configure capture parameters including resolution and pixel format
- Capture video frames for validation, debugging, or simple data dumping,
  depending on platform support and build configuration

Usage
=====

``nxcamera`` is an interactive command-line program. Start it from NSH, then
enter commands at the ``nxcamera>`` prompt:

.. code-block:: console

   nsh> nxcamera
   nxcamera>

Tip: Execute NSH Commands (Hidden)
==================================

At the ``nxcamera>`` prompt you can run an NSH command by prefixing it with
``!``. This is useful for quickly checking system state or invoking other
utilities without exiting ``nxcamera``.

.. code-block:: console

   nxcamera> !ls /dev
   /dev:
    console
    fb0
    gpio0
    gpio1
    gpio2
    gpio3
    loop
    null
    oneshot
    ram0
    ram1
    ram2
    video0
    video1
    zero
   nxcamera> !poweroff
   bash>

A typical workflow at the prompt is:

- ``input /dev/video0`` to set the input video node.
- ``output /dev/fb0`` to set the output node, for example a framebuffer.
- ``stream 640 480 30 NV12`` to start streaming with ``width height fps
  format``.
- ``stop`` to stop streaming.

You may just copy-paste the commands below to get started:

.. code-block:: console

  nxcamera
  input /dev/video0
  output /dev/fb0
  stream 640 480 30 YUYV # or NV12 on macOS

Pixel Format
============

For the ``stream`` command, the pixel format depends on the platform. On the
macOS ``sim`` platform you may use ``NV12``, while on Linux systems ``YUYV``
is more commonly used.

Examples
========

1. Start ``nxcamera`` and configure a typical interactive capture session:

.. code-block:: console

   nsh> nxcamera
   nxcamera> input /dev/video0
   nxcamera> output /dev/fb0
   nxcamera> stream 640 480 30 NV12
   nxcamera> stop

.. figure:: nxcamera_macos_sim.png
   :alt: nxcamera running on the macOS SIM platform
   :align: center

   ``nxcamera`` using the macOS AVFoundation backend on the SIM platform.

Features and Updates
====================

- Multiple camera instances are supported, allowing several cameras to be
  exposed as different device nodes such as ``/dev/video0`` and
  ``/dev/video1``. This makes it easier to select and validate different
  video input sources on the same system.
- On the ``sim`` platform, support has been added for the macOS
  AVFoundation backend. This enables camera capture and functional
  verification on macOS hosts, subject to build configuration and host
  permission settings.
