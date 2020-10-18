===============
NuttShell (NSH)
===============

The NuttShell is a very complete shell system to be used in NuttX, similar to bash and other similar options. It supports a rich set of included commands, scripting and the ability to run your own applications as "builtin" (part of the same NuttX binary). NSH is implemented as an application where most of the functionality is part of the library called `nshlib`.

As such, NSH is completely optional and can be disabled so that NuttX directly starts a given task instead of the main ``nsh`` application.

.. toctree::
  :maxdepth: 2
  :caption: Contents

  nsh.rst
  commands.rst
  config.rst
  customizing.rst
  builtin.rst
  installation.rst
  login.rst
