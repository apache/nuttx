=======================
``gemmlowp`` gemmlowp
=======================

``apps/math/gemmlowp`` vendors Google `gemmlowp
<https://github.com/google/gemmlowp>`_, a small low-precision matrix
library used by TensorFlow Lite Micro.

Enable ``CONFIG_MATH_GEMMLOWP``. The build downloads a pinned snapshot
(``719139ce755a0f31cbf1c37f7f98adcc7fc9f425``). Headers are added from
``apps/math/gemmlowp/gemmlowp``.

:doc:`../../mlearning/tflite-micro/index` depends on this package.
