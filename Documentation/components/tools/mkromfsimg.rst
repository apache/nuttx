=================
``mkromfsimg.sh``
=================

This script may be used to automate the generation of a ROMFS file system
image. It accepts an rcS script "template" and generates an image that
may be mounted under /etc in the NuttX pseudo file system.

.. tip::

   Edit the resulting header file and mark the generated data values as
   ``const`` so that they will be stored in FLASH.
