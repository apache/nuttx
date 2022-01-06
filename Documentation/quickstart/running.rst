.. include:: /substitutions.rst
.. _running:

=======
Running
=======

In order to finally run NuttX on your board, you first have to flash the NuttX
binary. As an easy start, it is recommended that you choose a well supported board
which also integrates the debugger/programmer in the board itself exposed via USB
connector.

A good choice is a Nucleo or Discovery board from ST Microelectronics,
as there is a wide choice of supported boards for the STM32 architecture in NuttX.
Also, these boards expose an UART port over the USB connection which allows you
to interact with NuttX via the interactive console without any extra hardware.
For the purposes of this guide, we will use the Nucleo F103RB board.

Flashing
========

There are various tools you can use to flash the NuttX binary to your Nucleo
board. One common option is to use ``openocd`` which supports a large number
of programmers and target microcontrollers.

You should note that ``openocd`` project has not made stable releases for long
time and support for newer hardware will probably be only available in the
latest Git version, so it is actually recommended to install latest development
version.

.. tabs::

  .. code-tab:: console Install stable version (Ubuntu)

     $ apt install openocd

  .. tab:: Install latest version from source

     .. code-block:: console

        $ git clone git://git.code.sf.net/p/openocd/code openocd
        $ cd openocd
        $ ./bootstrap
        $ ./configure --prefix=install/
        $ make install

     The resulting installation will be under ``openocd/install``. You can add
     ``openocd/install/bin`` to your ``PATH``.

Now, to flash the binary to your board, connect the USB cable and do:

.. code-block:: console

  $ cd nuttx/
  $ openocd -f interface/st-link-v2.cfg -f target/stm32f1x.cfg -c 'init' \
    -c 'program nuttx/nuttx.bin verify reset' -c 'shutdown'

Access NuttShell
================

Once you flash your board, it will reset and offer a prompt over the serial
console. With the Nucleo board, you can simply open the terminal program
of your choice where you will see the ``nsh>`` prompt (press :kbd:`enter`
if you don't see anything):

.. tabs::

  .. code-tab:: console picocom (CLI)

    $ picocom -b 115200 /dev/ttyUSB0

  .. code-tab:: console gtkterm (GUI)

    $ gtkterm -s 115200 -p /dev/ttyUSB0

.. tip::

  You may have to add yourself to the ``dialout`` group on Linux to have permission
  to access serial ports:

  .. code-block:: console

    $ gpasswd -a <user> dialout

  Where ``<user>`` is your username. You will need to log out from your desktop
  for the change to have effect.
