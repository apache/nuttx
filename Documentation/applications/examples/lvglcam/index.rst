.. _lvglcam:

=================================
``lvglcam`` LVGL Camera Dashboard
=================================

LVGL dashboard with the live view of a V4L2 camera and live system data.
It uses the V4L2 capture interface and the LVGL NuttX display driver only,
so it runs on any board that registers a camera as ``/dev/video`` and a
framebuffer or LCD for LVGL.

The camera format is the first one that LVGL draws directly (RGB565,
XRGB32 or BGR24).  The frame size is the largest one that fits the view, or
``CONFIG_EXAMPLES_LVGLCAM_WIDTH`` x ``CONFIG_EXAMPLES_LVGLCAM_HEIGHT``.  The
frames are captured into MMAP buffers and shown without a copy.  The view
and the data column are side by side on landscape displays and stacked on
portrait ones.

The system card (uptime, CPU load, free memory) is always shown.  Optional
cards, hidden when their device does not open:

==============================  ===========================================
Option                          Card
==============================  ===========================================
``EXAMPLES_LVGLCAM_ACCEL``      uORB accelerometer: roll, pitch and a chart
``EXAMPLES_LVGLCAM_NET``        network interface link state and address
``EXAMPLES_LVGLCAM_DEPTH``      V4L2 Z16 depth camera heatmap, up to 16x16
``EXAMPLES_LVGLCAM_SNAPSHOT``   button that saves the shown frame as
                                ``IMG_nnnn.BMP``
==============================  ===========================================

Start it in the background, SIGINT or SIGTERM stops it::

  nsh> mount -t vfat /dev/mmcsd0 /mnt
  nsh> lvglcam &

The CPU load needs ``/proc`` mounted and ``CONFIG_SCHED_CPULOAD_*``.  See
``stm32n6570-dk:lvglcam`` for a board configuration.
