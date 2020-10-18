.. _nx-graphics-library-nxgl-1:

==============================
NX Graphics Library (``NXGL``)
==============================

NXGL provides many APIs, some available for use internally by NX and
others for use by applications as well. Only those APIs intended for
application usage are documented here See ``include/nuttx/nx/nxglib.h``
for the full set of APIs; those APIs might be of interest if you are
rendering directly into framebuffer or LCD memory.

NXGL Types
----------

.. c:type:: nxgl_mxpixel_t

  Holds one device pixel. NXGLIB will select the
  smallest size for the ``nxgl_mxpixel_t`` that just contains the pixel:
  ``byte`` if 16, 24, and 32 resolution support is disabled, ``uint16_t``
  if 24, and 32 resolution support is disabled, or ``uint32_t``.

.. c:type:: nxgl_coord_t

  A given coordinate is limited to the screen height an
  width. If either of those values exceed 32,767 pixels, then the
  following will have to need to change:

.. c:struct:: nxgl_point_s

  Describes a point on the display:

  .. code-block:: c

    struct nxgl_point_s
    {
      nxgl_coord_t x;         /* X position, range: 0 to screen width - 1 */
      nxgl_coord_t y;         /* Y position, range: 0 to screen height - 1 */
    };

.. c:struct:: nxgl_size_s

  Describes the size of a rectangular region.

  .. code-block:: c

    struct nxgl_size_s
    {
      nxgl_coord_t w;        /* Width in pixels */
      nxgl_coord_t h;        /* Height in rows */
    };

.. c:struct:: nxgl_rect_s

  Describes a positioned rectangle on the display.

  .. code-block:: c

    struct nxgl_rect_s
    {
      struct nxgl_point_s pt1; /* Upper, left-hand corner */
      struct nxgl_point_s pt2; /* Lower, right-hand corner */
    };

.. c:struct:: nxgl_run_s

  Describes a run, i.e., a horizontal line. Note
  that the start/end positions have fractional precision. This is
  necessary for good joining of trapezoids when a more complex shape is
  decomposed into trapezoids.

  .. code-block:: c

    struct nxgl_run_s
    {
      b16_t        x1;        /* Left X position, range: 0 to x2 */
      b16_t        x2;        /* Right X position, range: x1 to screen width - 1 */
      nxgl_coord_t y;         /* Top Y position, range: 0 to screen height - 1 */
    };

.. c:struct:: nxgl_trapezoid_s

  Describes a horizontal trapezoid on the
  display in terms the run at the top of the trapezoid and the run at the
  bottom.

  .. code-block:: c

    struct nxgl_trapezoid_s
    {
      struct nxgl_run_s top;  /* Top run */
      struct nxgl_run_s bot;  /* bottom run */
    };

.. c:function:: void nxgl_rgb2yuv(uint8_t r, uint8_t g, uint8_t b, uint8_t *y, uint8_t *u, uint8_t *v)

  Convert 8-bit RGB triplet to 8-bit YUV triplet.

.. c:function:: void nxgl_yuv2rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);

  Convert 8-bit YUV triplet to 8-bit RGB triplet.

.. c:function:: void nxgl_rectcopy(FAR struct nxgl_rect_s *dest, FAR const struct nxgl_rect_s *src)

  This is essentially ``memcpy()``\ for rectangles. We
  don't do structure assignments because some compilers are not good at
  that.

.. c:function:: void nxgl_rectoffset(FAR struct nxgl_rect_s *dest, \
                     FAR const struct nxgl_rect_s *src, \
                     nxgl_coord_t dx, nxgl_coord_t dy);

  Offset the rectangle position by the specified dx, dy
  values.

.. c:function:: void nxgl_vectoradd(FAR struct nxgl_point_s *dest, \
                    FAR const struct nxgl_point_s *v1, \
                    FAR const struct nxgl_point_s *v2);


 Add two 2x1 vectors and save the result to a third.

.. c:function:: void nxgl_vectsubtract(FAR struct nxgl_point_s *dest, \
                       FAR const struct nxgl_point_s *v1, \
                       FAR const struct nxgl_point_s *v2);

  Add subtract vector ``v2`` from vector ``v1`` and
  return the result in vector dest.

.. c:function:: void nxgl_rectintersect(FAR struct nxgl_rect_s *dest, \
                        FAR const struct nxgl_rect_s *src1, \
                        FAR const struct nxgl_rect_s *src2);

  Return the rectangle representing the intersection of
  the two rectangles.

.. c:function:: void nxgl_rectunion(FAR struct nxgl_rect_s *dest, \
                    FAR const struct nxgl_rect_s *src1, \
                    FAR const struct nxgl_rect_s *src2);

  Given two rectangles, ``src1`` and ``src2``, return the
  larger rectangle that contains both, ``dest``.

.. c:function:: void nxgl_nonintersecting(FAR struct nxgl_rect_s result[4], \
                     FAR const struct nxgl_rect_s *rect1, \
                     FAR const struct nxgl_rect_s *rect2);

  Return the regions of rectangle ``rect1`` that do not
  intersect with ``rect2``. This will four rectangles, some of which may
  be degenerate (and can be picked off with :c:func:`nxgl_nullrect`).

.. c:function:: bool nxgl_rectoverlap(FAR struct nxgl_rect_s *rect1, \
                      FAR struct nxgl_rect_s *rect2);

  Return true if the two rectangles overlap.

.. c:function:: bool nxgl_rectinside(FAR const struct nxgl_rect_s *rect, \
                     FAR const struct nxgl_point_s *pt);

  Return true if the point ``pt`` lies within ``rect``.

.. c:function:: void nxgl_rectsize(FAR struct nxgl_size_s *size, \
                   FAR const struct nxgl_rect_s *rect);

  Return the size of the specified rectangle.

.. c:function:: bool nxgl_nullrect(FAR const struct nxgl_rect_s *rect);

  Return true if the area of the retangle is <= 0.

.. c:function:: void nxgl_runoffset(FAR struct nxgl_run_s *dest, \
                    FAR const struct nxgl_run_s *src, \
                    nxgl_coord_t dx, nxgl_coord_t dy);

  Offset the run position by the specified ``dx``, ``dy``
  values.

.. c:function:: void nxgl_runcopy(FAR struct nxgl_run_s *dest, \
                  FAR const struct nxgl_run_s *src);

  This is essentially ``memcpy()``\ for runs. We don't do
  structure assignments because some compilers are not good at that.

.. c:function:: void nxgl_trapoffset(FAR struct nxgl_trapezoid_s *dest, \
                     FAR const struct nxgl_trapezoid_s *src, \
                     nxgl_coord_t dx, nxgl_coord_t dy);

  Offset the trapezoid position by the specified ``dx``,
  ``dy`` values.

.. c:function:: void nxgl_trapcopy(FAR struct nxgl_trapezoid_s *dest, \
                   FAR const struct nxgl_trapezoid_s *src);

  This is essentially ``memcpy()``\ for trapezoids. We
  don't do structure assignments because some compilers are not good at
  that.

.. c:function:: void nxgl_colorcopy(nxgl_mxpixel_t dest[CONFIG_NX_NPLANES], \
               const nxgl_mxpixel_t src[CONFIG_NX_NPLANES]);

  This is essentially ``memcpy()``\ for colors. This does
  very little for us other than hide all of the conditional compilation
  for planar colors in one place.

.. c:function:: int nxgl_splitline(FAR struct nxgl_vector_s *vector, FAR struct nxgl_trapezoid_s *traps, \
                   FAR struct nxgl_rect_s *rect, nxgl_coord_t linewidth);

  In the general case, a line with width can be
  represented as a parallelogram with a triangle at the top and bottom.
  Triangles and parallelograms are both degenerate versions of a
  trapezoid. This function breaks a wide line into triangles and
  trapezoids. This function also detects other degenerate cases:

  #. If ``y1 == y2`` then the line is horizontal and is better represented
     as a rectangle.
  #. If ``x1 == x2`` then the line is vertical and also better represented
     as a rectangle.
  #. If the width of the line is 1, then there are no triangles at the top
     and bottom (this may also be the case if the width is narrow and the
     line is near vertical).
  #. If the line is oriented is certain angles, it may consist only of the
     upper and lower triangles with no trapezoid in between. In this case,
     3 trapezoids will be returned, but traps[1] will be degenerate.

  :param vector: A pointer to the vector described the line to be drawn.
  :param traps: A pointer to a array of trapezoids (size 3).
  :param rect: A pointer to a rectangle.

  :return:
    - 0: Line successfully broken up into three trapezoids. Values in traps[0], traps[1], and traps[2] are valid.
    - 1: Line successfully represented by one trapezoid. Value in traps[1] is valid.
    - 2: Line successfully represented by one rectangle. Value in rect is valid
    - <0: On errors, a negated errno value is returned.

.. c:function:: void nxgl_circlepts(FAR const struct nxgl_point_s *center, nxgl_coord_t radius, \
                    FAR struct nxgl_point_s *circle);

  Given a description of a circle, return a set of 16
  points on the circumference of the circle. These points may then be used
  by :c:func:`nx_drawcircle` or related APIs to draw a
  circle outline.

  :param center: A pointer to the point that is the center of the circle.
  :param radius: The radius of the circle in pixels.
  :param circle: A pointer the first entry in an array of 16 points where the circle points will be returned.

.. c:function:: void nxgl_circletraps(FAR const struct nxgl_point_s *center, nxgl_coord_t radius, \
                     FAR struct nxgl_trapezoid_s *circle);


  Given a description of a a circle, return 8 trapezoids
  that can be used to fill the circle by
  :c:func:`nx_fillcircle` and other interfaces.

  :param center: A pointer to the point that is the center of the circle.
  :param radius: The radius of the circle in pixels.
  :param circle: A pointer the first entry in an array of 8 trapezoids where the
    circle description will be returned.

