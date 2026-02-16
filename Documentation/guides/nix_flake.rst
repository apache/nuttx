======================================
Nix Flake for Reproducible Development
======================================

This guide explains how to use the Nix flake to set up a reproducible
development environment for NuttX. The Nix flake ensures that all required
build tools and dependencies are consistently available, simplifying
onboarding and reducing "works on my machine" issues.

Prerequisites
-------------

* `Nix <https://nixos.org/download/>`_ installed on your system.
* Nix flakes enabled (add ``experimental-features = nix-command flakes``
  to your ``nix.conf``).

Setting up the Development Environment
--------------------------------------

To enter the **NuttX build** development shell, navigate to the root of
the NuttX directory and run:

.. code-block:: bash

    nix develop ./tools/nix

This command will:

* Download and set up all necessary build tools and dependencies, including:

  * CMake, Ninja, GNU Make
  * Clang tools
  * ARM toolchain (gcc-arm-embedded)
  * Automake, Bison, Flex, Genromfs, Gettext, Gperf
  * Kconfig-frontends, libelf, expat, gmp, isl, libmpc, mpfr, ncurses, zlib
  * Python with kconfiglib

* Set the ``CMAKE_EXPORT_COMPILE_COMMANDS`` environment variable to ``ON``.
* Display a welcome message.

Once inside the development shell, you can proceed with building NuttX as usual.

Setting up the Documentation Environment
-----------------------------------------

The flake also provides a **documentation** development shell that includes
Sphinx and all required extensions for building the NuttX documentation.

To enter the documentation shell, run:

.. code-block:: bash

    nix develop ./tools/nix#docs

This command will:

* Provide Python 3.13 with Sphinx and extensions, including:

  * sphinx-rtd-theme
  * myst-parser
  * sphinx-tabs
  * sphinx-autobuild
  * sphinx-copybutton
  * sphinx-togglebutton
  * sphinx-design
  * sphinx-tags
  * sphinx-collapse
  * pytz, importlib-metadata

* Allow you to build and preview the documentation without installing
  anything globally.

Example usage:

.. code-block:: bash

    # Build HTML docs (has to but run in Documentation/ directory)
    make html

See the :doc:`/contributing/documentation` guide for more details.

Benefits
--------

* **Reproducibility:** Ensures a consistent build environment across all
  developers and machines.
* **Simplified Onboarding:** New contributors can quickly set up their
  development environment with a single command.
* **Dependency Management:** All dependencies are managed by Nix, avoiding
  conflicts with system-wide packages.
* **Separate Environments:** Keep firmware build tools and documentation
  tools isolated, or combine them if needed.

Contents of the Nix Flake
-------------------------

The `flake.nix` file defines two `devShells`:

* ``devShells.default`` - NuttX build environment:

.. code-block:: nix

    buildInputs = [
      # Build tools
      pkgs.cmake
      pkgs.ninja
      pkgs.gnumake
      pkgs.clang-tools

      # ARM toolchain
      pkgs.gcc-arm-embedded

      # NuttX dependencies
      pkgs.automake
      pkgs.bison
      pkgs.flex
      pkgs.genromfs
      pkgs.gettext
      pkgs.gperf
      pkgs.kconfig-frontends
      pkgs.libelf
      pkgs.expat.dev
      pkgs.gmp.dev
      pkgs.isl
      pkgs.libmpc
      pkgs.mpfr.dev
      pkgs.ncurses.dev
      pkgs.zlib
      pkgs.python313Packages.kconfiglib
    ];

    shellHook = ''
      export CMAKE_EXPORT_COMPILE_COMMANDS=ON
      echo "Welcome to NuttX devShell"
    '';

* ``devShells.docs`` - Documentation environment (from `Documentation/flake.nix`):

.. code-block:: nix

    python.withPackages (ps: with ps; [
      sphinx
      sphinx_rtd_theme
      myst-parser
      sphinx-tabs
      sphinx-autobuild
      sphinx-copybutton
      sphinx-togglebutton
      sphinx-design
      sphinx-tags
      sphinx-collapse
      pytz
      importlib-metadata
    ]);

This setup ensures that both firmware development and documentation building
are fully reproducible and isolated.
