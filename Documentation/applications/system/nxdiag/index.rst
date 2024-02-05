================================
``nxdiag`` NuttX Diagnostic Tool
================================

The NuttX Diagnostic Tool (Nxdiag) is a command line tool that can be used to gather information about the NuttX and host systems.
It also can be used to run a tests to verify that the vendor's tools are properly installed and configured.

Its primary purpose is to gather information that can be used to debug problems and ease the process of reporting bugs for uninexperienced users.

This tool uses a Python script (``apps/tools/host_sysinfo.py``) to gather information about the host system during build and a C
program to gather information about the NuttX system and display all available information. For more information about the python
script, check the command line options and code comments of ``host_sysinfo.py``.

.. note:: Nxdiag requires Python 3.6 or later. On Linux distributions, the ``distro`` Python module is
          recommended as it provides more accurate information about the host system.

Usage
-----

This page shows ``nxdiag`` options. Note that some options are only available if the
respective configuration options are enabled (see :ref:`cmdtable <nxdiagcmddependencies>`).
For a complete list of ``nxdiag`` options available to the system, just run ``nxdiag``::

    Usage: nxdiag [options]
    Options:
            -h                                 Show this message
            -n, --nuttx                        Output the NuttX operational system information.
            -f, --flags                        Output the NuttX compilation and linker flags used.
            -c, --config                       Output the NuttX configuration options used.
            -o, --host-os                      Output the host system operational system information.
            -p, --host-path                    Output the host PATH environment variable.
            -k, --host-packages                Output the host installed system packages.
            -m, --host-modules                 Output the host installed Python modules.
            -v, --vendor-specific              Output vendor specific information.
            --all                              Output all available information.

An example output can be observed `here <https://pastebin.com/HSw1EvhR>`_.

.. toctree::
  :maxdepth: 2
  :caption: Contents

  config.rst
