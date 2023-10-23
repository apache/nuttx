Porting Layer
=============

nimBLE supports being built as part of different OS, not only their mynewt
RTOS. A porting layer was written for NuttX, which was mostly a copy of
the Linux porting layer.

Modifying the porting layer
---------------------------

NuttX is supported in nimBLE by adding an entry in the porting layer
used to support different OSs. However, nimBLE supports each OS
by generating a configuration header (``syscfg.h``) from YAML configuration
files. If you want to modify the porting layer and change its configuration
you will need to regenerate this header. This process is a bit involved since
nimBLE uses its own ``newt`` build tool to do so and also somewhat assumes it will
be built for their mynewt OS, so it actually may fail to build completely but
it will still get to generate the required files.

So, first is to get the newt tool::

  $ cd apps/nimble
  $ git clone https://github.com/apache/mynewt-newt
  $ cd mynewt-newt

At the moment, you will probably require unstable version
instead of a release so select a known working::

  $ git checkout c14c47bb683d
  $ ./build.sh

There should be now a ``newt`` binary under ``mynewt-newt/newt``.
Extend your path so that it is visible::

  $ export PATH=mynewt-newt/newt:$PATH

Now, create a ``newt`` project::

  $ newt new foo

We want latest master version of mynewt OS and stack, so edit
``foo/project.yml`` and change the ``vers`` variable to ``0.0.0``. Now
do::

  $ cd foo/
  $ newt upgrade

Under ``foo/repos`` there will be a clone of both mynewt and nimble
repo. Since this app already downloads nimble repo outside of ``foo``,
you can delete ``foo/repos/apache-mynewt-nimble`` and simply make a
link to the ``mynewt-nimble`` directory, so that you can work on the
nimBLE code directly.

Now you can make any changes to the ``yml`` files such as
``porting/targets/nuttx/syscfg.yml``. Finally, you can build with::

  $ newt build @apache-mynewt-nimble/porting/targets/nuttx

This will most likely fail to complete but the generated headers
should be there. So now copy them to the appropriate location in
the ``nuttx`` target directory::

  $ cd foo/
  $ cp bin/@apache-mynewt-nimble/porting/targets/nuttx/generated/include/logcfg/logcfg.h \
	repos/apache-mynewt-nimble/porting/examples/nuttx/include/logcfg
  $ cp bin/@apache-mynewt-nimble/porting/targets/nuttx/generated/include/syscfg/syscfg.h \
	repos/apache-mynewt-nimble/porting/examples/nuttx/include/syscfg

If these changes are done to fix a problem with NuttX porting layer in nimBLE, you
should open a pull-request to nimBLE repository to include the updated header files.
It is recommended to mention the issue in NuttX mailing list first to ensure the change
is needed.
