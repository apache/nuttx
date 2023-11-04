===============
Host Side Tools
===============

``bitmap_converter.py`` NxWidgets
=================================

This script converts from any image type supported by Python imaging library to
the RLE-encoded format used by NxWidgets.

RLE (Run Length Encoding) is a very simply encoding that compress quite well
with certain kinds of images: Images that that have many pixels of the same
color adjacent on a row (like simple graphics). It does not work well with
photographic images.

But even simple graphics may not encode compactly if, for example, they have
been resized. Resizing an image can create hundreds of unique colors that may
differ by only a bit or two in the RGB representation. This "color smear" is the
result of pixel interpolation (and might be eliminated if your graphics software
supports resizing via pixel replication instead of interpolation).

When a simple graphics image does not encode well, the symptom is that the
resulting RLE data structures are quite large. The palette structure, in
particular, may have hundreds of colors in it. There is a way to fix the graphic
image in this case. Here is what I do (in fact, I do this on all images prior to
conversion just to be certain):

- Open the original image in GIMP.
- Select the option to select the number of colors in the image.
- Pick the smallest number of colors that will represent the image faithfully.
  For most simple graphic images this might be as few as 6 or 8 colors.
- Save the image as PNG or other lossless format (NOT jpeg).
- Then generate the image.

``mkromfsimg.sh``
=================

**Q**: Why are there two versions of the script ``mkromfsimg.sh``, one in
``apps/tools`` and one in ``nuttx/tools``.

**A**: The version of ``mkromfsimg.sh`` in ``nuttx/tools`` is a generic
tool to simplify creation of ROMFS file system from any directory contain
content that you would like to access within the the target.

The version in ``apps/tools``, on the other hand, has a very special purpose.
It is part of the support that can be used in the KERNEL build mode.

Processes and Programs in the KERNEL Build
------------------------------------------

In the kernel build, there are no tasks. There are only processes and all
code lives in its own, private address space.
See :doc:`/implementation/processes_vs_tasks`.

One consequence of that is that functions like ``task_create()`` and friends
cannot be used in the KERNEL build mode. Instead, all processes must be loaded
into a virtual address space from an ELF or NxFLAT file residing in the file
system. ROMFS is one of many file system, but one that is particularly usable
for this purpose in deeply embedded systems.

KERNEL Build Differences
------------------------

In the FLAT and PROTECTED build mode all applications are built into a single
BLOB, so every symbol must have a unique name to avoid name collisions.

In the KERNEL build mode, all applications are built at separately linked
programs that reside in a file system. The entry point to ALL programs is the
function ``main()``.

apps/bin
--------

When you build the ``apps/`` programs in FLAT or PROTECTED modes, all of the
object files are put into an archive apps/libapps.a which is, eventually,
copied to ``nuttx/libs`` and the BLOB is created by linking NuttX archives
with ``lib/libapps.a``.

But when you build the ``apps/`` programs in the KERNEL mode, the directory
``apps/bin`` is created by the top-level apps/Makefile. Each source file is
compiled, but the object files are not added to ayn archive. Instead, the
object files are linked into a separate compiled and linked program. Each program
is then installed at ``apps/bin``.

apps/tools/mkromfsimg.sh
------------------------

When the ``apps/`` kernel build is complete, all of the programs have been installed
in ``apps/bin``. That is where ``apps/tools/mkromfsimg.sh`` file comes into to play.
It takes all of the programs in apps/bin and creates a ROMFS file system image
containing all of the applications. That ROMFS file system image is built into
the kernel.

Application Initialization
--------------------------

At run time, when the kernel boots, it will mount that ROMFS file system at ``/bin``.
In the FLAT build mode, the OS boot logic calls ``task_create()`` to start the initial
task you have configured with ``CONFIG_INIT_ENTRYPOINT``. But in the KERNEL build, something
different happens. ``CONFIG_INIT_ENTRYPOINT`` is not used. Instead, ``CONFIG_INIT_FILEPATH``
is used. This will be the name of the program to stared in ``/bin`` to bring up the system.
