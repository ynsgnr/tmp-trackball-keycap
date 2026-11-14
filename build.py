"""Keycap-mounted replacement base for the HG66 mini ring trackball.

Replaces the original finger-ring base: an open box that holds the PCB and
battery, takes the original top shell (hinge tongue at one end, two screws at
the other) and is fused onto an MX cross-stem low-profile 1u keycap (1u.stl).

Run: python3 build.py  ->  trackball_mount.stl  (mm)
Print upright (stem down) with supports from the build plate only.

Axes (top view): X = long axis, +X = screw end, -X = hinge end
                 Y = short axis, -Y = USB-C/switch side, +Y = battery side
Keycap sits under the -X/+Y corner, so the box only overhangs towards +X and
-Y (off the keyboard).  Mirror in the slicer for the opposite corner.
All shell numbers are estimated from photos - check the ones marked (*).
"""
import cadquery as cq, trimesh

# ---- top shell (estimated) ----
L, W, R   = 37.0, 30.0, 9.0   # outer outline of shell rim, corner radius
TOP_WALL  = 1.0               # top-shell wall thickness at the rim
SCREW_DY  = 17.5              # (*) distance between the two screw posts
SCREW_EDGE = 4.4              # (*) screw centre to the +X end
SCREW_D, HEAD_D = 2.2, 3.6    # clearance hole / head counterbore
SEAT_T    = 2.0               # (*) material under the screw head (= screw length - ~4)
TONGUE_W  = 9.0               # (*) hinge tongue width on the top shell
TONGUE_T  = 1.3               # (*) hinge tongue thickness
# ---- PCB (estimated) ----
PCB_T     = 1.0
PCB_DROP  = 5.7               # (*) seam (base rim) down to PCB top face
PCB_GAP   = 1.6               # clearance under the PCB (USB-C shell, parts)
PCB_HOLES = [(5.6, 4.85), (-6.1, -0.8)]   # (*) PCB locating holes (x, y)
PEG_D     = 1.3
PCB_Y_BAT = 7.1               # PCB edge on the battery side
PCB_X_HINGE = -14.2           # PCB edge at the hinge end
WIN_X     = (-12.0, 11.5)     # combined USB-C + power switch opening along X
PLUG_X    = (-12.0, 2.0)      # floor relief under the USB-C plug overmold
# ---- box ----
WALL, FLOOR = 1.6, 1.5
LIP_H, LIP_T, LIP_CLR = 1.2, 0.9, 0.3
# ---- keycap ----
KEY, CAP_TOP, OVERLAP = 18.0, 4.8, 0.5

z0   = CAP_TOP - OVERLAP              # underside of floor
zf   = z0 + FLOOR                     # floor top
zpb  = zf + PCB_GAP                   # PCB underside
seam = zpb + PCB_T + PCB_DROP         # rim: top shell sits here
H    = seam - z0

def slab(x0, x1, y0, y1, za, zb):
    return (cq.Workplane("XY").workplane(offset=za)
            .center((x0 + x1) / 2, (y0 + y1) / 2).rect(x1 - x0, y1 - y0).extrude(zb - za))

def cyl(x, y, d, za, zb):
    return cq.Workplane("XY").workplane(offset=za).center(x, y).circle(d / 2).extrude(zb - za)

outer  = cq.Workplane("XY").workplane(offset=z0).rect(L, W).extrude(H).edges("|Z").fillet(R)
cavity = (cq.Workplane("XY").workplane(offset=zf).rect(L - 2 * WALL, W - 2 * WALL)
          .extrude(H).edges("|Z").fillet(R - WALL))
box = outer.cut(cavity)

# screw bosses at +X end, counterbored from below
sx = L / 2 - SCREW_EDGE
for sy in (SCREW_DY / 2, -SCREW_DY / 2):
    box = box.union(cyl(sx, sy, HEAD_D + 1.6, z0, seam).intersect(outer))
for sy in (SCREW_DY / 2, -SCREW_DY / 2):
    box = box.cut(cyl(sx, sy, SCREW_D, z0 - 1, seam + 1)).cut(cyl(sx, sy, HEAD_D, z0 - 1, seam - SEAT_T))

# PCB supports: shouldered pegs, edge rail on battery side, pad at hinge end
for x, y in PCB_HOLES:
    box = box.union(cyl(x, y, 3.2, zf, zpb)).union(cyl(x, y, PEG_D, zpb, zpb + PCB_T + 0.4))
box = box.union(slab(1, 11, PCB_Y_BAT - 0.9, PCB_Y_BAT + 0.5, zf, zpb))
box = box.union(slab(PCB_X_HINGE - 0.3, PCB_X_HINGE + 1.0, -3, 3, zf, zpb))

# locating lip on the battery side (fits inside the top shell wall)
yo = W / 2 - TOP_WALL - LIP_CLR; yi = yo - LIP_T; yw = W / 2 - WALL + 0.2
lip = (cq.Workplane("YZ").polyline([(yw, seam - 2.0 - (yw - yi)), (yi, seam - 2.0),
       (yi, seam + LIP_H), (yo, seam + LIP_H), (yo, seam), (yw, seam)]).close().extrude(16)
       .translate((-8, 0, 0)))
box = box.union(lip)

# hinge end: pocket for the top-shell tongue + window under a rim bridge for its catch
tw = TONGUE_W + 1.0
box = box.cut(slab(-L / 2 + 0.8, -L / 2 + 0.8 + TONGUE_T + 0.8, -tw / 2, tw / 2, seam - 4.5, seam + 1))
box = box.cut(slab(-L / 2 - 1, -L / 2 + 1, -tw / 2, tw / 2, seam - 4.5, seam - 1.5))

# USB-C + switch: wall opened up to the rim, floor relieved under the plug
box = box.cut(slab(WIN_X[0], WIN_X[1], -W / 2 - 1, -W / 2 + WALL + 0.01, zf, seam + 1))
box = box.cut(slab(PLUG_X[0], PLUG_X[1], -W / 2 - 1, -W / 2 + 2.8, z0 - 1, zf + 1))

cq.exporters.export(box, "/tmp/_box.stl", tolerance=0.01, angularTolerance=0.05)
cap = trimesh.load("1u.stl")
c = cap.bounds.mean(axis=0)
cap.apply_translation([-c[0] - L / 2 + KEY / 2, -c[1] + W / 2 - KEY / 2, 0])
out = trimesh.boolean.union([cap, trimesh.load("/tmp/_box.stl")], engine="manifold")
print("seam z", round(seam, 2), "bounds", out.bounds.round(2).tolist(),
      "watertight", out.is_watertight, "bodies", out.body_count)
out.export("trackball_mount.stl")
