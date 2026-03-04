================
NuttX CI Process
================

NuttX is a complex system with lots of compile-time configurable switches and
values. What's more is that NuttX supports hundreds of different hardware
targets, across multiple different architectures. This complexity makes it
critical to have CI processes for testing incoming patches.

.. note::

   The NuttX CI resources are limited, and not everything in the kernel can be
   reasonably tested in CI (for instance, there is currently no support for
   testing directly on hardware targets via CI, although it's being worked on).

   It is for this reason that NuttX still request patch authors perform their
   own local testing (especially on hardware where applicable) to submit
   alongside their patch so reviewers may have some confidence that the change
   will not introduce obvious regressions.

.. tip::

   NuttX is always appreciative of improvements to our processes! If you have
   suggestions to improve the CI infrastructure, please let the community know.
   Our CI team is currently bearing a high workload.

The focus of this documentation is the CI testing that takes place when a PR is
opened on the `nuttx <https://github.com/apache/nuttx>`_ (and `nuttx-apps
<https://github.com/apache/nuttx-apps>`_) GitHub repositories.

CI Stages
=========

When a PR is opened, the following CI actions take place:

* Pull request labeller assigns labels to the PR
* PR is checked using the host tool :doc:`checkpatch.sh
  </components/tools/checkpatch>`
* Linting is performed using `Super-Linter
  <https://github.com/super-linter/super-linter/pkgs/container/super-linter>`_
* Build tests are performed based on which files were modified
* Some CI runtime tests are performed on simulators/emulators through
  :doc:`/testing/ntfc`.

Pull Request Labelling
======================

This action is responsible for:

* Labelling the PR with the label(s) corresponding to changed files (i.e. files
  changed under ``arch/arm/**`` are labelled ``Arch: arm``).

* Labelling the PR with a PR size (i.e. ``Size: XS``, ``Size: M``, etc.)

The workflow file for this action is located at
``.github/workflows/labeler.yml``. This file contains documentation in the form
of comments. To compute the PR labels, it:

* Gets information about the changes from GitHub (i.e. files and lines changed)
* Sums the total lines changed and assigns PR size labels based on this number
* Uses the wildcard paths-to-label assignments in `.github/labeler.yml` to
  assign the correct change labels to PRs.

Types of labels are:

* Arch labels (i.e. ``Arch: arm``, ``Arch: risc-v``, etc), associated with
  changes in ``arch/``
* Board labels (i.e. ``Board: arm``, ``Board: sim``, etc) associated with
  changes in ``boards/``
* Area labels (i.e. ``Area: Bluetooth``, ``Area: Crypto``, ``Area: Drivers``)
  associated with several different kernel "areas"

Once this workflow is done running, the PRs labels are updated by the computed
result and PRs in the pull-requests tab can be sorted according to these labels.

.. tip::

   You can filter PRs by the ``Size: XS``, ``Size: S`` to review small PRs
   quickly in between work :)

Checkpatch
==========

More information about the ``checkpatch.sh`` tool itself can be found at
:doc:`/components/tools/checkpatch`.

The goal of this action is to verify that the PR adheres to the :doc:`C coding
standard </contributing/coding_style>`, does not contain typos and adheres to
the :doc:`commit message format </contributing/making-changes>`. Additionally,
format checking of CMake files has been added to the checks.

The workflow file for this action is very short and can be found at
``.github/workflows/check.yml``.

Build
=====

This is the most complex step in the CI process, and also the longest duration.
It selects a category of NuttX configurations (``defconfig`` files) associated
with the changed files and builds them all in parallel. It also uses the
``tools/refresh.sh`` utility to check if any configurations need to be
normalized (see :doc:`/components/tools/refresh`) for more information).

The workflow file for this check is located at ``.github/workflows/build.yml``.
The name of this workflow is deceiving, as it does not only build
configurations, but also normalizes them and runs :doc:`NTFC </testing/ntfc>`
tests on architectures that support it.

The steps executed in the build workflow are:

1. Fetch source: checks out ``nuttx`` and ``nuttx-apps`` repos. The source files
   are added as an artifact to the GitHub action so that it can be downloaded by
   subsequent steps.

2. In parallel, the builds to be performed are selected for each host OS/setup
   supported by NuttX. These are Linux, MacOS, MSVC and MYSYS2.

   .. note::

      Typically only performed in the Linux environment. Performing the builds
      across all host options consumes too many resources.

      MacOS builds are currently always skipped, but sometimes tests are
      performed for MSVC and MYSYS2. These builds are a small subset of the
      total builds performed on Linux, but which were selected to still cover a
      range of architectures. Some selected builds are ``raspberrypi-pico:nsh``,
      ``rv-virt:nsh``, ``sim:windows``, etc.

3. Once the build is selected for a host configuration (i.e. Linux), the builds
   are performed in parallel. For example, if in Step 2 the selected Linux
   builds were ``arm-01``, ``arm-02`` and ``arm-03``, this step will perform the
   build test for each category in parallel.

   This involves compiling all of the ``defconfig`` configurations included in
   each category and running ``tools/refresh.sh`` on them. Some categories, like
   ``sim-01`` also perform NTFC tests. In this case, the NTFC runtime tests are
   run using the architecture's designated ``citest/defconfig`` configuration.

4. After each category's builds are complete, `make host_info` is run in each
   category's runner as a sanity check.

5. After all the selected Linux builds complete, an out-of-tree (OOT) build test
   is performed. This is done using ``tools/ci/cibuild-oot.sh``.

Each build category (``arm-01``, etc.) produces a build artifact which contains
all of the resulting build outputs for that category. This includes the
resulting ``nuttx.bin``, ``nuttx.elf``, ``nuttx.hex``, etc., binaries. You
should be able to download the artifact, flash the NuttX image(s) to your target
device and test them that way if you'd like to avoid building all of the images
yourself!
