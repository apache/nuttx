=========================================
NTFC (NuttX Test Framework for Community)
=========================================

NTFC enables automated testing for NuttX across :doc:`simulator
</platforms/sim/sim/index>`, QEMU, and real hardware via serial interface. The
framework automatically detects available applications in NuttX images and
executes applicable tests using pytest-based test cases.

The framework and official test cases are available at:

- https://github.com/apache/nuttx-ntfc
- https://github.com/apache/nuttx-ntfc-testing

Detailed documentation is available in the framework repository.

CI migration to NTFC is in progress. Configurations currently using NTFC:

- :doc:`risc-v/qemu-rv </platforms/risc-v/qemu-rv/index>`: ``rv-virt/citest64``
- :doc:`risc-v/qemu-rv </platforms/risc-v/qemu-rv/index>`: ``rv-virt/citest``
- :doc:`sim/sim </platforms/sim/sim/index>`: ``sim/citest``
- :doc:`arm/imx6 </platforms/arm/imx6/index>`: ``sabre-6quad/citest``
- :doc:`arm64/qemu </platforms/arm64/qemu/boards/qemu-armv8a/index>`: ``qemu-armv8a/citest``
- :doc:`arm64/qemu </platforms/arm64/qemu/boards/qemu-armv8a/index>`: ``qemu-armv8a/citest_smp``

NTFC exports test logs as CI artifacts. This allows test logs (including
:doc:`ostest </applications/testing/ostest/index>` output) to be downloaded
directly from the CI for all targets where the ``citest`` configuration is
enabled.
