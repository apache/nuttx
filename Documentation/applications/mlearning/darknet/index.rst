============================================
``darknet`` YOLO: Real-Time Object Detection
============================================

``apps/mlearning/darknet`` vendors `Darknet
<https://github.com/pjreddie/darknet>`_, the framework behind YOLO (You
Only Look Once) object detection.

Enable ``CONFIG_DARKNET_YOLO``. ``CONFIG_DARKNET_YOLO_VER`` is the Git
branch fetched at build time (default ``master``). The Makefile compiles
the core Darknet sources (network, parser, convolutional and connected
layers, YOLO/region/detection layers, RNN/GRU/LSTM, and image helpers).

Include path: ``apps/mlearning/darknet/darknet/include``.

Darknet is a larger, floating-point oriented stack compared with
TensorFlow Lite Micro. Prefer :doc:`../tflite-micro/index` for
microcontroller-class inference.
