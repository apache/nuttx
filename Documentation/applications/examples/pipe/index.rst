=====================
``pipe`` Pipe example
=====================

A test of the ``mkfifo()`` and ``pipe()`` APIs. Requires ``CONFIG_PIPES``

- ``CONFIG_EXAMPLES_PIPE_STACKSIZE`` – Sets the size of the stack to use when
  creating the child tasks. The default size is ``1024``.
