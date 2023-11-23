.. include:: /substitutions.rst
.. _citests:

=======================
Running CI Test Locally
=======================

NuttX automatically runs continuous integration (CI) tests on 
`simulator <https://nuttx.apache.org/docs/latest/guides/simulator.html>`__
target when new pull request is submitted. To avoid the tests failing you can
also run them locally on your computer prior to submiting new pull request.
This page describes the step by step manual to do so.

Configuring NuttX
=================

NuttX has a simulator target that allows the user to run NuttX as a regular
program on a computer. The simulator target with CI test is configured and
compiled followingly.

  .. code-block:: console

      $ cd nuttx
      $ ./tools/configure.sh sim:citest
      $ make

Now you can run the simulator to check the configuration was successful.

  .. code-block:: console

      $ ./nuttx
      login: admin
      password: Administrator

You should see NuttX shell with built in test applications. Now you can exit
the simulator.

  .. code-block:: console

      nsh> poweroff
      $
      $ # we're back at the Linux prompt.

Running CI Tests
================

Running CI tests locally requires Minicom and Python 3.6 or newer to be
installed on the system. Other requirements can be installed with following
set of commands.

  .. code-block:: console

      $ cd tools/ci/testrun/env
      $ pip3 install -r requirements.txt
      $ cd ..
      $ cd script

Now you have everything prepared to run CI tests. The tests themself are run
by following command.

  .. code-block:: console

      $ python3 -m pytest -m 'common or sim' ./ -B sim -P nuttx-path -L log-path -R sim -C --json=log-path/pytest.json

Where nuttx-path is an absolute path to NuttX root directory and log-path is
a user defined directory to which tests log are saved.
