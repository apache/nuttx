=============================
``flatbuffers`` FlatBuffers
=============================

``apps/system/flatbuffers`` vendors Google `FlatBuffers
<https://github.com/google/flatbuffers>`_ ``v23.5.26`` and applies
``flatbuffers.patch``. TensorFlow Lite Micro uses the headers to read
``.tflite`` model schemas.

Enable ``CONFIG_SYSTEM_FLATBUFFERS``. Include path:
``apps/system/flatbuffers/flatbuffers/include``.

:doc:`../../mlearning/tflite-micro/index` depends on this package.
