# Camera intrinsics (Basler, 8 mm lens, Mono8)

Recovered from a 9x6 inner-corner checkerboard, 20 mm squares, 39/40 views.
RMS reprojection error: 0.16 px.

Camera matrix K:
    fx = 1443.2497   fy = 1443.7377
    cx =  963.2151   cy =  580.3333

Distortion (k1, k2, p1, p2, k3):
    [-0.1575447, 0.16190211, -0.00044674, -0.00022088, -0.11840788]

These values are hard-coded in code/basler_apriltag_multitag_fusion.py.
Recalibrate and update them if the lens, focus, aperture, or resolution changes.
