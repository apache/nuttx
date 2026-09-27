===============
Video Subsystem
===============

Everything to do with moving images: cameras that produce frames, frame
buffers that display them, and the interfaces in between.

Where the code is
=================

Unlike most sections of this documentation, "video" is not one directory --
and the directory named after it is the smallest part of it.

``video/`` at the top of the source tree carries ``CONFIG_VIDEO``, the master
switch, but it is not only a Kconfig stub: ``video/videomode/`` under it is a
small library of its own -- ``edid_parse.c``, ``edid_dump.c``, ``vesagtf.c``,
``videomode_lookup.c``, ``videomode_sort.c`` -- enabled separately by
``CONFIG_VIDEO_EDID``.  The ``dummy.c`` beside it is there so the directory
still compiles to something when nothing under it is selected.

Everything else is elsewhere.  ``drivers/video/`` holds the working code for
V4L2 and the frame buffer -- ``v4l2_core.c``, ``v4l2_cap.c``, ``v4l2_m2m.c``,
``fb.c``, ``video_framebuff.c`` -- alongside the per-chip drivers, and
``include/nuttx/video/`` holds the interfaces those parts agree on.

One header is not in either place, and it is the one an application needs: the
V4L2 API is ``include/sys/videoio.h``, in the POSIX-style location rather than
under ``nuttx/``, mirroring where Linux puts ``videodev2.h``.  That is the file
to ``#include``.  The headers below are the internal contracts.

The interfaces
==============

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Header
     - What it defines
   * - ``sys/videoio.h``
     - *Not in this directory.*  The V4L2 API itself: the ``VIDIOC_`` ioctls
       and the ``v4l2_*`` types, 1665 lines of them.  Being the Linux API
       means an application that already speaks V4L2 needs little changing.
   * - ``v4l2_cap.h``, ``v4l2_m2m.h``, ``video.h``
     - The internal side of V4L2: capture, and memory-to-memory transforms.
       ``v4l2_cap.h`` is the join between the API and the two camera halves
       below, and includes both of them; ``v4l2_m2m.h`` and ``video.h``
       include ``sys/videoio.h`` directly.
   * - ``imgsensor.h``, ``imgdata.h``
     - The two halves a camera port implements: the sensor being configured,
       and the path the pixels take out of it.
   * - ``fb.h``
     - The frame buffer interface, for the display side.
   * - ``mipi_dsi.h``, ``mipi_display.h``
     - MIPI DSI, for displays attached over that link.
   * - ``videomode.h``, ``edid.h``, ``vesagtf.h``
     - Timings and mode descriptions, including reading a mode out of a
       monitor's EDID.  These three are the exception to the paragraph above:
       they are implemented in ``video/videomode/``, not under ``drivers/``.
   * - ``rfb.h``, ``vnc.h``
     - The remote frame buffer protocol, for a display that is somewhere
       else entirely.

The rest of ``include/nuttx/video/`` is per-chip headers -- ``ov2640.h``,
``isx012.h``, ``isx019.h``, ``gc0308.h``, ``max7456.h`` and the three
``goldfish_*`` headers -- plus ``rgbcolors.h``, which is colour-conversion
macros rather than an interface.

Drivers
=======

The device drivers themselves -- the sensors, the frame buffers, the
display controllers -- are documented with the rest of the drivers:

* :doc:`/os/drivers/special/video` for video device drivers;
* :doc:`/os/graphics/index` for what draws into a frame buffer once it
  exists.
