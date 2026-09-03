========================================
``libnnablart`` NNABLA Runtime Libraries
========================================

``apps/mlearning/libnnablart`` builds the Sony `NNABLA C Runtime
<https://github.com/sony/nnabla-c-runtime>`_ so NuttX applications can run
networks produced by Neural Network Libraries.

Enable ``CONFIG_NNABLA_RT``. ``CONFIG_NNABLA_RT_VER`` selects the upstream
tag (default ``1.24.0``). The Makefile downloads that release and compiles
the runtime plus a subset of operators, including:

- Activations (ReLU, sigmoid, tanh, softmax, ELU, leaky ReLU, Swish, ...)
- Arithmetic and reduction
- Affine (float and fixed-point)
- Convolution, depthwise convolution, deconvolution, and pooling
- Batch normalization, dropout, reshape, concatenate, and related array ops
- Binary-weight / binary-connect quantization helpers

Include path: ``apps/mlearning/libnnablart/nnabla-c-runtime/include``.

CMSIS 5 NN kernels with the NNABLA CHW helpers can be enabled together
via :doc:`../cmsis/index`.
