======
Erbium
======

Erbium is an AIFoundry RISC-V platform with a 64-bit Minion core. This
initial port targets the ``erbium_emu`` system emulator from
`ET-platform <https://github.com/aifoundry-org/et-platform>`_.

The port runs in machine mode on hart 0, with a flat address space and SMP
disabled. Other harts remain parked and never initialize NuttX memory.
Supported peripherals are the machine timer, PLIC and UART0 console.
Silicon, protected/kernel builds, SMP, storage, networking
and other peripherals have not been validated by this port.

.. toctree::
   :maxdepth: 1

   boards/minion/index

The emulator currently only stores writes to the system reset register; it
does not reboot the simulated system. The board therefore does not expose
``BOARDIOC_RESET`` or claim reset support.
