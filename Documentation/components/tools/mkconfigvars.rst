===================
``mkconfigvars.sh``
===================

The HTML documentation expects to have a copy of the auto-generated
configuration variable documentation Documentation/NuttXConfigVariables.html.
The script mkconfigvars.sh is a simple script that can be used to
re-generated that file as needed.

Help:

.. code:: console

   $ tools/mkconfigvars.sh -h
   tools/mkconfigvars.sh is a tool for generation of configuration variable documentation

   USAGE: tools/mkconfigvars.sh [-d|h] [-v <major.minor.patch>]

Where::

    -v <major.minor.patch>
       The NuttX version number expressed as a major, minor and patch number separated
       by a period
    -d
       Enable script debug
    -h
       show this help message and exit
