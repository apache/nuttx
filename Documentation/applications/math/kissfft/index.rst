===================
``kissfft`` KissFFT
===================

``apps/math/kissfft`` vendors `KissFFT
<https://github.com/mborgerding/kissfft>`_ (tag ``v130``) and applies
``kissfft.patch``. It compiles ``kiss_fft.c`` and ``tools/kiss_fftr.c``.

Enable ``CONFIG_MATH_KISSFFT``. Include path:
``apps/math/kissfft/kissfft``.

:doc:`../../mlearning/tflite-micro/index` depends on this package.
