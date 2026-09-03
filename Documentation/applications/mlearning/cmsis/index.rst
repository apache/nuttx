=========================
``cmsis`` CMSIS Libraries
=========================

``apps/mlearning/cmsis`` fetches `CMSIS 5
<https://github.com/ARM-software/CMSIS_5>`_ and builds CMSIS-DSP and
CMSIS-NN from that tree. Default version is ``5.8.0``
(``CONFIG_CMSIS_VER``).

This package is aimed at NNABLA and generic DSP use. For TensorFlow Lite
Micro kernel acceleration, use :doc:`../cmsis-nn/index` instead.

Enable ``CONFIG_CMSIS`` from :menuselection:`Application Configuration -->
Machine Learning Support --> CMSIS Libraries`.

``CONFIG_CMSIS_DSP``
  CMSIS-DSP. Optional ``CONFIG_CMSIS_DSP_ARM_MATH_MATRIX_CHECK`` and
  ``CONFIG_CMSIS_DSP_ARM_MATH_ROUNDING`` map to ``ARM_MATH_MATRIX_CHECK``
  and ``ARM_MATH_ROUNDING``. ``CONFIG_ARCH_FPU`` defines
  ``__FPU_PRESENT``.

``CONFIG_CMSIS_NN``
  CMSIS-NN from the CMSIS 5 tree. Depends on ``CONFIG_CMSIS_DSP``. A
  NuttX patch (``cmsis-nn-support_nnabla.patch``) adds CHW convolution
  helpers used by NNABLA.

Public headers are added from:

- ``CMSIS_5/CMSIS/Core/Include``
- ``CMSIS_5/CMSIS/DSP/Include`` (when DSP is enabled)
- ``CMSIS_5/CMSIS/NN/Include`` (when NN is enabled)

See https://arm-software.github.io/CMSIS_5/General/html/index.html.
