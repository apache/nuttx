===================
``flash_writer.py``
===================

This flash writer is using the xmodem for firmware transfer on
boards based on cxd56 chip (Ex. Spresense).  This tool depends on
the xmodem package (https://pypi.org/project/xmodem/).

For flashing the ``.spk`` image to the board please use:

.. code:: console

   $ tools/flash_writer.py -s -c /dev/ttyUSB0 -d -b 115200 -n nuttx.spk
