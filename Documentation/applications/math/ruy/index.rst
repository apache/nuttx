=============
``ruy`` Ruy
=============

``apps/math/ruy`` vendors Google `Ruy <https://github.com/google/ruy>`_,
a matrix-multiplication library used by TensorFlow Lite Micro.

Enable ``CONFIG_MATH_RUY``. The build downloads a pinned snapshot
(``d37128311b445e758136b8602d1bbd2a755e115d``). Headers are added from
``apps/math/ruy/ruy``.

:doc:`../../mlearning/tflite-micro/index` depends on this package.
