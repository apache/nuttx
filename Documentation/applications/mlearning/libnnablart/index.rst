========================================
``libnnablart`` NNABLA Runtime Libraries
========================================

``apps/mlearning/libnnablart`` builds the Sony `NNABLA C Runtime
<https://github.com/sony/nnabla-c-runtime>`_ so NuttX applications can run
networks produced by Neural Network Libraries.

Enable ``CONFIG_NNABLA_RT``. ``CONFIG_NNABLA_RT_VER`` selects the upstream
tag (default ``1.24.0``). Sony's later ``nnabla-c-runtime`` tags go
through ``v1.38.0``; NuttX has not moved the default. Sony has placed
NNABLA in maintenance. There is no NuttX example application and no
CMake build. The Makefile downloads the selected release and compiles
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
