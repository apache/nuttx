==========================================
``libjpeg`` JPEG image encoding library
==========================================

``libjpeg`` is an open-source library that provides functionality for encoding and decoding images in the JPEG format.  
It implements the baseline JPEG standard and is commonly used in image-related applications and utilities.

Including the library
=====================

To use ``libjpeg`` in a C program, include the main header:

.. code-block:: c

   #include <jpeglib.h>

Configuration
=============

Support for ``libjpeg`` must be enabled in Kconfig:

  CONFIG_LIBJPEG=y

Example
=======

NuttX provides an example application that demonstrates how to encode, decode, and resize JPEG images using ``libjpeg``.  
It can be found under:

``apps/graphics/jpgresizetool``
