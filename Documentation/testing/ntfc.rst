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

CI runtime test cases use NTFC. Configurations with an NTFC runner include:

- :doc:`risc-v/qemu-rv </platforms/risc-v/qemu-rv/index>`: ``rv-virt/citest64``
- :doc:`risc-v/qemu-rv </platforms/risc-v/qemu-rv/index>`: ``rv-virt/citest``
- :doc:`sim/sim </platforms/sim/sim/index>`: ``sim/citest``
- :doc:`arm/imx6 </platforms/arm/imx6/index>`: ``sabre-6quad/citest``
- :doc:`arm64/qemu </platforms/arm64/qemu/boards/qemu-armv8a/index>`: ``qemu-armv8a/citest``
- :doc:`arm64/qemu </platforms/arm64/qemu/boards/qemu-armv8a/index>`: ``qemu-armv8a/citest_smp``

Running a CI Test Target Locally
================================

Run NTFC from the top-level NuttX source directory.  The relative paths in a
target's ``config.yaml`` file, including the NuttX configuration and image
paths, are resolved from that directory.  A working NuttX build environment
and any emulator or hardware required by the target must already be available.

Create a Python 3.10 or newer virtual environment and install the NTFC and test
case versions currently used by NuttX CI:

.. code-block:: console

   $ python3 -m venv ../ntfc-venv
   $ . ../ntfc-venv/bin/activate
   $ python -m pip install ntfc==0.0.1
   $ git clone --branch release-0.0.2 --depth 1 \
       https://github.com/apache/nuttx-ntfc-testing ../nuttx-ntfc-testing

Build the target in the usual way.  For example, the simulator CI
configuration uses the Make build:

.. code-block:: console

   $ ./tools/configure.sh sim:citest
   $ make

Configurations whose ``config.yaml`` refers to ``./build/nuttx`` require an
out-of-tree CMake build instead.  For example, build ``qemu-armv8a:citest``
with:

.. code-block:: console

   $ cmake -B build -DBOARD_CONFIG=qemu-armv8a:citest -GNinja
   $ cmake --build build

Then pass the test case checkout and the target's NTFC configuration files to
NTFC.  For ``sim:citest`` this is:

.. code-block:: console

   $ python -m ntfc test \
       --testpath=../nuttx-ntfc-testing \
       --confpath=boards/sim/sim/sim/configs/citest/config.yaml \
       --jsonconf=boards/sim/sim/sim/configs/citest/session.json

For another target, replace the build target and both configuration paths with
the files from that target's ``configs/<configuration>`` directory.  NTFC
selects the applicable tests from the NuttX configuration and image and stores
the test results under ``result/`` by default.

NTFC exports test logs as CI artifacts. This allows test logs (including
:doc:`ostest </applications/testing/ostest/index>` output) to be downloaded
directly from the CI for all targets where the ``citest`` configuration is
enabled.
