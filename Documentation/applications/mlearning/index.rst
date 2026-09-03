========================
Machine Learning Support
========================

NuttX machine learning support lives in the `apps
<https://github.com/apache/nuttx-apps>`_ repository under
``apps/mlearning``. These packages wrap upstream TinyML runtimes and
optimized kernels. The build system fetches the selected upstream sources
when the corresponding Kconfig option is enabled.

Enable the libraries from::

  Application Configuration  --->
    Machine Learning Support  --->

The following packages are available:

- :doc:`cmsis/index` — ARM CMSIS 5 DSP and NN libraries
- :doc:`cmsis-nn/index` — standalone ARM CMSIS-NN kernels for TensorFlow Lite Micro
- :doc:`darknet/index` — Darknet YOLO object detection
- :doc:`libnnablart/index` — Sony NNABLA C inference runtime
- :doc:`tflite-micro/index` — TensorFlow Lite for Microcontrollers

A ready-to-build simulator configuration is ``sim:tflm``. See
:doc:`/platforms/sim/sim/boards/sim/index`.

.. toctree::
   :glob:
   :maxdepth: 1
   :titlesonly:
   :caption: Contents

   */index*
