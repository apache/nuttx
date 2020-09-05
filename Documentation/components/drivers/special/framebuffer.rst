====================
Frame Buffer Drivers
====================

-  ``include/nuttx/video/fb.h``. All structures and APIs
   needed to work with frame buffer drivers are provided in this
   header file.

-  ``struct fb_vtable_s``. Each frame buffer device driver
   must implement an instance of ``struct fb_vtable_s``. That
   structure defines a call table with the following methods:

   Get information about the video controller configuration and
   the configuration of each color plane.

   The following are provided only if the video hardware supports
   RGB color mapping:

   The following are provided only if the video hardware supports
   a hardware cursor:

-  **Binding Frame Buffer Drivers**. Frame buffer drivers are not
   normally directly accessed by user code, but are usually bound
   to another, higher level device driver. In general, the binding
   sequence is:

   #. Get an instance of ``struct fb_vtable_s`` from the
      hardware-specific frame buffer device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``arch/sim/src/up_framebuffer.c``. See also the
   usage of the frame buffer driver in the ``graphics/``
   directory.
