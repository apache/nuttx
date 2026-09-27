=============
API Reference
=============

The POSIX and NuttX interfaces an application may call.  This answers "what
can I call, and what does it do"; for how the OS behind those calls works,
see :doc:`/os/index`.

The interfaces that run the other way -- the ones a *port* has to supply to
NuttX, rather than the ones an application calls -- are filed with the code
that needs them: :doc:`/os/arch/arch_api` for what architecture-specific
logic exports, and :doc:`/os/arch/board_api` for what board-specific logic
exports.

.. toctree::
   :caption: Contents:
   :maxdepth: 1

   user/index.rst
