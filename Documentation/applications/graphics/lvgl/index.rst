=============
``lvgl`` LVGL
=============

Usage
-----

Import with ``#include <lvgl/lvgl.h>`` or ``#include <lvgl.h>``.

Upstream example ported to NuttX is present at ``examples/lvgldemo``.

LVGL can be used with framebuffer device. To find example boards with this
preconfigured, search for ``CONFIG_GRAPHICS_LVGL=y`` in ``defconfig`` files. All of
them have also ``CONFIG_VIDEO_FB=y`` present.

As a second option, LVGL can talk to a display driver and explicitly draw line
by line. For this case, there is no preconfigured board present. Go to _Porting_
section of upstream documentation for more hints.

Resources
---------

- `API documentation with examples <https://docs.lvgl.io/latest/en/html/index.html>`_
- `GitHub / LVGL / LVGL library <https://github.com/lvgl/lvgl>`_
- `GitHub / LVGL / Examples, tutorials, applications <https://github.com/lvgl/lv_examples>`_
- `GitHub / LVGL / Desktop simulator <https://github.com/lvgl/lv_sim_eclipse_sdl>`_
- `GitHub / LVGL / Web simulator <https://github.com/lvgl/lv_sim_emscripten>`_
