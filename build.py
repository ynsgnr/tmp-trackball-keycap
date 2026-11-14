"""Trackball-on-a-keycap mount: replaces the finger-ring base of the HG66 mini
trackball with a plate fused onto an MX cross-stem low-profile 1u keycap (1u.stl).

Run: python3 build.py   ->  trackball_mount.stl  (units: mm)
Print plate-side down (flip 180 deg), no supports needed.

Layout (top view, keycap stem up = keyboard side is -Z):
  plate is centred on the origin; trackball shell sits on top of it.
  -X long side = USB-C / power switch side (faces off the keyboard)
  -Y end       = the two shell screws      (faces off the keyboard)
  keycap sits under the +X/+Y corner, so nothing overhangs neighbouring keys.
Mirror in the slicer for the opposite corner of the board.
"""
import cadquery as cq, trimesh

# ---- measure these on your shell and adjust (estimated from the scan) ----
SHELL_W   = 30.0   # outer width of shell opening (short axis, X)
SHELL_L   = 37.0   # outer length (long axis, Y)
SHELL_R   = 12.0   # corner radius of the outline
SCREW_DX  = 17.6   # centre-to-centre distance of the two shell screws
SCREW_EDGE = 4.2   # screw centre to the -Y end of the shell
SCREW_D   = 2.0    # clearance hole
HEAD_D    = 4.2    # screw-head counterbore diameter
HEAD_H    = 1.0    # counterbore depth
# ---- mount ----
PLATE_T   = 2.5
KEY       = 18.0   # keycap footprint
CAP_TOP   = 4.8    # top of 1u.stl
OVERLAP   = 0.5    # plate sinks this far into the keycap top (solid there)

z0 = CAP_TOP - OVERLAP
sy = -SHELL_L / 2 + SCREW_EDGE
pts = [(-SCREW_DX / 2, sy), (SCREW_DX / 2, sy)]
plate = (cq.Workplane("XY").workplane(offset=z0)
         .rect(SHELL_W, SHELL_L).extrude(PLATE_T)
         .edges("|Z").fillet(SHELL_R)
         .faces("<Z").workplane().pushPoints([(x, -y) for x, y in pts])
         .cboreHole(SCREW_D, HEAD_D, HEAD_H))
cq.exporters.export(plate, "/tmp/_plate.stl", tolerance=0.01, angularTolerance=0.05)

cap = trimesh.load("1u.stl")
c = cap.bounds.mean(axis=0)
cap.apply_translation([-c[0] + SHELL_W / 2 - KEY / 2, -c[1] + SHELL_L / 2 - KEY / 2, 0])
out = trimesh.boolean.union([cap, trimesh.load("/tmp/_plate.stl")], engine="manifold")
print("bounds", out.bounds.round(2).tolist(), "watertight", out.is_watertight, "bodies", out.body_count)
out.export("trackball_mount.stl")
