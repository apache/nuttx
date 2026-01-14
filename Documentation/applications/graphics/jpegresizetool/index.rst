================
JPEG Resize Tool
================

A lightweight utility for resizing JPEG images using parameters from the ``libjpeg`` library.  
This tool serves both as a simple example of how to use ``libjpeg`` and as a practical scriptable resizer.

Usage
=====

.. code-block:: bash

   jpegresize <input.jpg> <output.jpg> <scale> <quality>

Arguments
---------

- **input.jpg** — Path to the input JPEG image.  
- **output.jpg** — Path to the resized output image.  
- **scale** — Shrinking denominator. Currently, ``libjpeg`` supports values ``1``, ``2``, ``4``, or ``8``.  
  For example, a value of ``8`` reduces the image dimensions by a factor of 8.  
- **quality** — JPEG quality factor from ``0`` to ``100``.  
  - Above **80%**: compression is nearly unnoticeable.  
  - Around **20%**: strong compression artifacts appear.

Example
=======

Shrink an image by 8× with 20% quality:

.. code-block:: bash

   jpegresize /sd/IMAGES/000000a1.jpg /sd/TEMP/THUMBS/000000a1.jpg 8 20

Notes
=====

- This example assumes ``libjpeg`` is available in your build environment.  
- The tool is designed for simplicity and small-footprint image processing on embedded systems or automation scripts.
