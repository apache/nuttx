=============================
``cmsis-nn`` CMSIS-NN Library
=============================

``apps/mlearning/cmsis-nn`` builds the standalone `CMSIS-NN
<https://github.com/ARM-software/CMSIS-NN>`_ kernels. This is the package
TensorFlow Lite Micro uses when ``CONFIG_MLEARNING_CMSIS_NN`` is enabled.

It is distinct from :doc:`../cmsis/index`, which vendors the older CMSIS 5
tree (DSP plus NN) for NNABLA.

Enable with ``CONFIG_MLEARNING_CMSIS_NN``. The Makefile downloads a pinned
CMSIS-NN commit and compiles:

- Activation functions
- Basic math
- Concatenation
- Convolution
- Fully connected
- LSTM
- NN support functions
- Pooling
- Reshape
- Softmax
- SVD

When ``CONFIG_ARM_NEON`` is also set, a few kernels are omitted here
because TFLM supplies NEON replacements under
``apps/mlearning/tflite-micro/operators/neon``.

Headers are added to the include path from
``apps/mlearning/cmsis-nn/cmsis-nn/Include``.

For TFLM, enabling this option defines ``CMSIS_NN`` and swaps TFLM
reference kernels for the CMSIS-NN variants. See
:doc:`../tflite-micro/index`.
