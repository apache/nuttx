========================================
``cxx-oot-build`` Out-of-Tree Build Test
========================================

The ``cxx-oot-build`` test automates building an **Out-of-Tree (OOT)** NuttX
project using a NuttX export tarball. Its primary purpose is to validate that
NuttX can be built outside of the main source tree and to prevent regressions
in the build process for C++ projects.

**Important:** This test uses a specialized defconfig that is **not functional**
for running actual applications. It is intended purely for CI/build
verification. Functional OOT projects should be configured according to
the instructions in :ref:`cpp_cmake`.

The test script is located at:

- ``tools/ci/cibuild-oot.sh``

### Out-of-Tree App Content

The source content for this OOT test can be found in:

- ``apps/testing/cxx-oot-build``

Its structure provides a basic skeleton for building a C++ NuttX application:

.. code-block:: text

    testing/cxx-oot-build
    ├── CMakeLists.txt
    ├── include
    │   └── HelloWorld.hpp
    └── src
        ├── HelloWorld.cpp
        └── main.cpp

This minimal structure includes:

- `CMakeLists.txt` - Build instructions for the OOT C++ project  
- `include/HelloWorld.hpp` - Example header file  
- `src/HelloWorld.cpp` - Example class implementation  
- `src/main.cpp` - Entry point for the test application  

### How to Run the Test

Execute the test script from the NuttX CI tools directory:

    cd ${NUTTX_PATH}/tools/ci
    ./cibuild-oot.sh

The script performs the following steps:

1. Configures NuttX for the ``cxx-oot-build`` board profile
2. Builds an export tarball of NuttX
3. Prepares the Out-of-Tree project by extracting the tarball
4. Builds the OOT project using CMake
5. Verifies that the output binaries ``oot`` and ``oot.bin`` exist

### Expected Output

On success, you should see:

    ✅ SUCCESS: OOT build completed. Output:
    -rwxrwxr-x 1 <user> <group> 94K <date> /path/to/oot
    -rwxrwxr-x 1 <user> <group> 46K <date> /path/to/oot.bin

If any step fails, the script will exit immediately with an error message.

### Notes

- No additional configuration options are required for this test. The
  ``cxx-oot-build`` defconfig is preconfigured to build correctly but is
  **not suitable for running applications**.
- For functional OOT builds, please follow the procedure documented in
  :doc:`here </guides/cpp_cmake>`.
