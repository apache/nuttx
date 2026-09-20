===================
``kissfft`` KissFFT
===================

KissFFT is a small open-source Fast Fourier Transform (FFT) library
("Keep It Simple, Stupid") used in many embedded projects. It provides
mixed-radix complex FFTs and a real-input helper, with floating-point
or fixed-point types and a BSD license that is easy to vendor.

``apps/math/kissfft`` vendors `KissFFT
<https://github.com/mborgerding/kissfft>`_ (tag ``v130``) and applies
``kissfft.patch``. It compiles ``kiss_fft.c`` and ``tools/kiss_fftr.c``.

Enable ``CONFIG_MATH_KISSFFT``. Include path:
``apps/math/kissfft/kissfft``.

:doc:`../../mlearning/tflite-micro/index` depends on this package.
