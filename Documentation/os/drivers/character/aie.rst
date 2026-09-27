========================
AI Engine (AIE) Drivers
========================

``drivers/aie`` is an upper-half character driver for hardware neural
processing units. Enable ``CONFIG_AI_ENGINE``. There is no in-tree
lower-half or board configuration yet; a platform must implement
``struct aie_ops_s`` and call ``aie_register()``.

The public header is ``include/nuttx/aie/ai_engine.h``. Ioctl commands:

- ``AIE_CMD_LOAD`` — load a model. The argument is a model pointer
  passed to ``ops->init()``. A second load on the same file returns
  ``-EINVAL``.
- ``AIE_CMD_FEED_INPUT`` — feed one input tensor via ``ops->feed_input()``.
- ``AIE_CMD_GET_OUTPUT`` — read one output tensor via ``ops->get_output()``.

Other ioctl numbers are forwarded to ``ops->control()`` when that
callback is provided, otherwise ``-ENOSYS``.

This kernel driver is independent of the application-level TinyML
packages under :doc:`/applications/mlearning/index`.
