"""Keycap-mounted replacement base for the HG66 mini ring trackball.

Replaces the original finger-ring base with an open box that
  * holds the PCB and the 501020 battery (standing on edge beside the PCB),
  * takes the original top shell: a hinge tab sticks out of the end wall at one
    end, two original screws go in from below at the other end,
  * is fused onto an MX cross-stem low-profile 1u keycap (1u.stl).

Run:   python3 build.py   ->  trackball_mount.stl/.step (with keycap),
       trackball_base_plain.stl/.step (box only), preview.svg   (units: mm)
Print: upright (stem down), 0.4 mm nozzle / 0.2 mm layers, supports from the
       build plate only.

Axes (top view):  X = long axis,  +X = screw end,  -X = hinge end
                  Y = short axis, -Y = USB-C / power switch side, +Y = battery
The keycap sits under the -X/+Y corner, so the box only overhangs towards +X
and -Y (off the keyboard).  Mirror in the slicer for the opposite corner.

The wall follows the top shell's rim outline in outline.json, traced from the
underside photo and scan by fit_outline.py.  Numbers marked (*) are estimated
from photos; the rest was measured.
"""
import json

import cadquery as cq
import trimesh
from shapely.affinity import translate
from shapely.geometry import Polygon, box as rect

# ---- FDM tolerances (0.4 mm nozzle, 0.2 mm layers) -------------------------
CLR      = 0.2    # clearance per mating face, horizontal and vertical
HOLE_CLR = 0.4    # added to the diameter of vertical holes (they print small)
# Printed features are kept at >= 0.8 mm (two extrusion widths) and all levels
# (floor, PCB seat, rim, tab) land on 0.2 mm layers.

# ---- top shell --------------------------------------------------------------
RIM = Polygon(json.load(open("outline.json")))   # rim outline traced from the
XH, YU, XS, YB = RIM.bounds                      # photo/scan by fit_outline.py
SCREW_DY   = 17.5              # (*) distance between the two screw posts
SCREW_EDGE = 4.4               # (*) screw centre to the +X end
SCREW_LEN  = 4.0               # screw length under the head
SCREW_HEAD = 2.0               # screw head diameter
SCREW_GRIP = 2.6               # thread left to bite into the top-shell post
DRIVER_D   = 3.2               # counterbore: head + screwdriver shaft
TAB_W      = 10.0              # hinge tab width (original base)
TAB_T      = 1.2               # lower hinge tab thickness below the rim plane
LEDGE_IN   = 2.4               # (*) shell hinge ledge: inner edge from the shell's outer face
LEDGE_T    = 1.0               # shell hinge ledge thickness
CLIP_UP_T  = 1.2               # upright joining the lower and upper hinge tabs
CLIP_TOP_T = 1.0               # upper hinge tab thickness
CLIP_GRIP  = 1.0               # how far the upper tab reaches back over the ledge

# ---- electronics ------------------------------------------------------------
PCB_T      = 1.0
PCB_DROP   = 5.8               # (*) rim down to the PCB top face
PCB_UNDER  = 3.0               # tallest part under the PCB (side button)
PCB_X      = (-14.2, 14.8)     # (*) PCB extent along X
PCB_Y      = (-11.75, 7.1)     # (*) PCB extent along Y
PCB_SHAPE  = [(-14.2, 6.3), (-13.4, 7.1), (11, 7.1), (11, 4.85), (14.8, 4.85), (14.8, -6.2),
              (11.2, -6.2), (11.2, -11.75), (-9.6, -11.75), (-14.2, -4.9)]   # (*)
PCB_HOLES  = [(5.6, 4.85), (-7.1, -0.8)]   # locating holes (second one moved after test fit)
PEG_D      = 0.8               # 1.2 printed too fat for the PCB holes
BATTERY    = (20.0, 5.2, 10.0) # length, width, height standing on edge
BAT_X      = -0.3              # battery centre along X (where the bay is widest)
WIN_X      = (-12.0, 11.5)     # (*) USB-C + power switch opening along X
SILL_H     = 4.0               # wall left under that opening, from the box underside
BTN_Y      = (-1.5, 5.5)       # (*) access to the underside button, +X wall

# ---- box --------------------------------------------------------------------
WALL, FLOOR = 1.2, 1.6         # 1.2 wall: the battery needs the room at the corners
LIP_H      = 1.3               # wall raised by this much in the lip areas
LIP_BOSS_X = 11.5              # battery-side lip stops here, short of the screw boss
LIP_END_Y  = 6.0               # screw-end lip spans |y| < this, between the bosses
HINGE_GAP  = 12.0              # unraised width at the hinge tab (tab + 1 mm each side)
SKIRT_OUT  = 1.2               # outer wall grows outwards by this, over its full height
SKIRT_H    = 1.0               # ...and rises this far above the wall top, round the shell

# ---- keycap (1u.stl: 18 x 18 x 4.8, flat top, 1 mm ceiling) ----------------
KEY, CAP_TOP, OVERLAP = 18.0, 4.8, 0.4

# ---- derived levels ---------------------------------------------------------
Z0   = CAP_TOP - OVERLAP                 # underside of the floor
ZF   = Z0 + FLOOR                        # floor top
ZPCB = ZF + PCB_UNDER + CLR              # PCB underside
SEAM = ZPCB + PCB_T + PCB_DROP           # rim; the top shell sits here


def slab(x, y, z):
    """Axis-aligned block from (min, max) pairs."""
    return (cq.Workplane("XY").workplane(offset=z[0])
            .center(sum(x) / 2, sum(y) / 2).rect(x[1] - x[0], y[1] - y[0])
            .extrude(z[1] - z[0]))


def cyl(x, y, d, z):
    return cq.Workplane("XY").workplane(offset=z[0]).center(x, y).circle(d / 2).extrude(z[1] - z[0])


def outline(inset, z):
    """Rim outline shrunk by `inset`, extruded over z = (min, max)."""
    ring = RIM.buffer(-inset, 32).exterior
    pts = [ring.interpolate(i / 96, normalized=True).coords[0] for i in range(96)]
    return cq.Workplane("XY").workplane(offset=z[0]).spline(pts, periodic=True).close().extrude(z[1] - z[0])


def shell():
    return outline(-SKIRT_OUT, (Z0, SEAM)).cut(outline(WALL, (ZF, SEAM + 1)))


def screw_bosses():
    """Bosses at the +X end; screws enter from below through a deep counterbore."""
    x = XS - SCREW_EDGE
    seat = SEAM - (SCREW_LEN - SCREW_GRIP)        # underside of the screw head
    add, cut = cq.Workplane("XY"), cq.Workplane("XY")
    for y in (SCREW_DY / 2, -SCREW_DY / 2):
        add = add.union(cyl(x, y, DRIVER_D + 2 * 0.8, (Z0, SEAM)))
        # thread is ~1.4: nominal 1.6 hole prints ~1.4-1.5 and still seats a 2.0 head
        cut = cut.union(cyl(x, y, SCREW_HEAD - HOLE_CLR, (Z0 - 1, SEAM + 1)))
        cut = cut.union(cyl(x, y, DRIVER_D, (Z0 - 1, seat)))
    return add.intersect(outline(0, (Z0, SEAM))), cut


def pcb_supports():
    s = slab((1, 11), (PCB_Y[1] - 1.0, PCB_Y[1]), (ZF, ZPCB))            # battery-side edge
    s = s.union(slab((PCB_X[0] - 0.4, PCB_X[0] + 1.0), (-3, 3), (ZF, ZPCB)))  # hinge-end edge
    for x, y in PCB_HOLES:
        s = s.union(cyl(x, y, 3.2, (ZF, ZPCB))).union(cyl(x, y, PEG_D, (ZPCB, ZPCB + PCB_T + 0.4)))
    return s


def hinge_clip():
    """C-shaped hinge at the -X end: the lower tab sticks out of the wall flush with
    the rim, an upright rises from its inner edge and an upper tab points back out.
    The shell's hinge ledge slides into the slot between the tabs (see SLIDE)."""
    y = (-TAB_W / 2 + CLR / 2, TAB_W / 2 - CLR / 2)
    x_up = XH + LEDGE_IN + CLR                      # upright, just inboard of the ledge
    z_slot = SEAM + LEDGE_T + 2 * CLR               # 0.2 fit + 0.2 for overhang sag
    lower = slab((XH + WALL - 0.1, x_up + CLIP_UP_T), y, (SEAM - TAB_T, SEAM))
    upright = slab((x_up, x_up + CLIP_UP_T), y, (SEAM - TAB_T, z_slot + CLIP_TOP_T))
    upper = slab((XH + LEDGE_IN - CLIP_GRIP, x_up + 0.1), y, (z_slot, z_slot + CLIP_TOP_T))
    return lower.union(upright).union(upper)


SLIDE = CLIP_GRIP + CLR + 0.1       # the shell goes on this far towards -X, then slides in


def slide_relief():
    """Removes whatever stands outside the shell above the rim plane near the hinge
    end, as far along as the shell needs to sit SLIDE mm outwards before sliding in."""
    shifted = translate(RIM, -SLIDE, 0).exterior
    pts = [shifted.interpolate(i / 2000, normalized=True) for i in range(2000)]
    x_end = max(p.x for p in pts if not RIM.buffer(CLR).contains(p)) + SLIDE + 0.5
    z = (SEAM, SEAM + 5)
    return slab((XH - SKIRT_OUT - 1, x_end), (YU - SKIRT_OUT - 1, YB + SKIRT_OUT + 1), z).cut(
        outline(0, (SEAM - 1, SEAM + 6))), x_end


def lips():
    """Everything above the rim plane.  The wall is raised by LIP_H in three stretches
    (hinge-end corner on the USB side, hinge round the battery side to the boss, and
    between the bosses).  Outside the shell an outer skirt stands SKIRT_H above the
    wall top everywhere, so it covers the seam."""
    top = SEAM + LIP_H + SKIRT_H
    z = (SEAM - 1, top + 1)
    raised = (slab((XH - 3, WIN_X[0]), (YU - 3, -HINGE_GAP / 2), z)
              .union(slab((XH - 3, LIP_BOSS_X), (HINGE_GAP / 2, YB + 3), z))
              .union(slab((XS - 6, XS + 3), (-LIP_END_Y, LIP_END_Y), z)))
    wall = outline(-SKIRT_OUT, (SEAM, SEAM + LIP_H)).cut(outline(WALL, z)).intersect(raised)
    skirt_ring = outline(-SKIRT_OUT, (SEAM, top)).cut(outline(-CLR, z))
    skirt = skirt_ring.intersect(raised).union(
        outline(-SKIRT_OUT, (SEAM, SEAM + SKIRT_H)).cut(outline(-CLR, z)))
    return wall.union(skirt)


def usb_opening():
    """Removes the -Y wall (and skirt) over WIN_X above the sill, through its full
    thickness, so the wall ends are clean faces even where the wall curves."""
    z = (Z0 + SILL_H, SEAM + LIP_H + SKIRT_H + 1)
    y = (YU - SKIRT_OUT - 1, YU + 6)
    wall_only = slab((XH - 3, XS + 3), y, z).cut(outline(WALL, (z[0] - 1, z[1] + 1)))
    return slab(WIN_X, y, z).intersect(wall_only)


def button_opening():
    return slab((XS - WALL - 1.0, XS + SKIRT_OUT + 1), BTN_Y, (ZF, ZPCB + 0.4))


def build():
    bosses, holes = screw_bosses()
    box = shell().union(bosses).union(pcb_supports()).union(hinge_clip()).union(lips())
    return box.cut(slide_relief()[0]).cut(holes).cut(usb_opening()).cut(button_opening())


def keycap_cad():
    """Native re-model of 1u.stl (for STEP export): 18 mm cap, straight skirt then a
    tapered top, 1 mm ceiling, 5.5 mm stem with a 4.4 x 1.42 mm cross."""
    def rrect(size, r, z):
        return cq.Sketch().rect(size, size).vertices().fillet(r).moved(cq.Location((0, 0, z)))
    def body(size, r, z_straight, z_top, slope=0.45):
        top = size - 2 * slope * (z_top - z_straight)
        low = cq.Workplane("XY").placeSketch(rrect(size, r, 0)).extrude(z_straight)
        return low.union(cq.Workplane("XY").placeSketch(rrect(size, r, z_straight),
                                                         rrect(top, r, z_top)).loft())
    cap = body(KEY, 1.5, 1.5, CAP_TOP).cut(body(KEY - 2.0, 0.5, 1.27, CAP_TOP - 1.0))
    stem = cyl(0, 0, 5.5, (0, CAP_TOP - 0.5))
    cross = slab((-2.2, 2.2), (-0.71, 0.71), (-1, CAP_TOP - 1.0)).union(
        slab((-0.71, 0.71), (-2.2, 2.2), (-1, CAP_TOP - 1.0)))
    return cap.union(stem.cut(cross)).translate((XH - SKIRT_OUT + KEY / 2, YB + SKIRT_OUT - KEY / 2, 0))


def validate(box):
    """Check that the PCB and battery have room and nothing sticks up past the lips."""
    solid = box.val()
    def overlap(other):
        return solid.intersect(other.val()).Volume()
    bat_l, bat_w, bat_h = BATTERY
    room = RIM.buffer(-WALL - CLR)                       # usable floor plan
    bat = rect(BAT_X - bat_l / 2, PCB_Y[1] + CLR, BAT_X + bat_l / 2, PCB_Y[1] + CLR + bat_w)
    pcb = cq.Workplane("XY").workplane(offset=ZPCB + 0.01).polyline(PCB_SHAPE).close().extrude(PCB_T - 0.01)
    pegs = len(PCB_HOLES) * 3.1416 * PEG_D ** 2 / 4 * (PCB_T - 0.01)
    checks = {
        "PCB outline does not fit inside the walls": not room.contains(Polygon(PCB_SHAPE)),
        "PCB body collides (pegs excluded)":         overlap(pcb) - pegs > 0.01,
        "under-PCB clearance < 3.2":                 ZPCB - ZF < PCB_UNDER + CLR - 1e-6,
        "battery does not fit inside the walls":     not room.contains(bat),
        "battery collides":
            overlap(slab((bat.bounds[0], bat.bounds[2]), (bat.bounds[1], bat.bounds[3]),
                         (ZF + 0.01, ZF + bat_h))) > 1e-3,
        "battery taller than cavity":                SEAM - ZF < bat_h - 1e-6,
        "something sticks up past the lips":         solid.BoundingBox().zmax > SEAM + max(LIP_H + SKIRT_H,
                                                LEDGE_T + 2 * CLR + CLIP_TOP_T) + 1e-6,
        "screw boss outside the rim outline":
            not RIM.buffer(-0.8).contains(rect(XS - SCREW_EDGE - 0.1, -SCREW_DY / 2,
                                               XS - SCREW_EDGE + 0.1, SCREW_DY / 2)),
    }
    for name, failed in checks.items():
        print(("FAIL  " if failed else "ok    ") + name)
    print(f"rim outline {XS - XH:.1f} x {YB - YU:.1f}; levels: floor {Z0:.1f}-{ZF:.1f}, "
          f"PCB seat {ZPCB:.1f}, rim {SEAM:.1f}; PCB-wall gap {room.exterior.distance(Polygon(PCB_SHAPE)) + CLR:.2f}, "
          f"battery-wall gap {room.exterior.distance(bat) + CLR:.2f}; shell slides in {SLIDE:.1f}, "
          f"outer lip removed for x < {slide_relief()[1]:.1f}")
    return not any(checks.values())


if __name__ == "__main__":
    box = build()
    ok = validate(box)
    cq.exporters.export(box, "/tmp/_box.stl", tolerance=0.01, angularTolerance=0.05)
    cq.exporters.export(box, "preview.svg", opt={"projectionDir": (0.5, -1.0, 2.0),
                                                 "showAxes": False, "width": 900, "height": 600})
    cap = trimesh.load("1u.stl")
    c = cap.bounds.mean(axis=0)
    cap.apply_translation([XH - SKIRT_OUT + KEY / 2 - c[0], YB + SKIRT_OUT - KEY / 2 - c[1],
                           -cap.bounds[0][2]])
    out = trimesh.boolean.union([cap, trimesh.load("/tmp/_box.stl")], engine="manifold")
    print("bounds", out.bounds.round(2).tolist(), "watertight", out.is_watertight,
          "bodies", out.body_count)
    assert ok and out.is_watertight and out.body_count == 1
    out.export("trackball_mount.stl")
    plain = trimesh.load("/tmp/_box.stl")            # box only, floor underside at z = 0
    plain.apply_translation([0, 0, -Z0])
    assert plain.is_watertight and plain.body_count == 1
    plain.export("trackball_base_plain.stl")
    print("plain bounds", plain.bounds.round(2).tolist())
    # native CAD (STEP) for SolidWorks etc.; the keycap there is a re-model of 1u.stl
    cq.exporters.export(box.translate((0, 0, -Z0)), "trackball_base_plain.step")
    kc = keycap_cad()
    print("keycap re-model volume %.1f vs 1u.stl %.1f mm3" % (kc.val().Volume(), cap.volume))
    cq.exporters.export(box.union(kc), "trackball_mount.step")
