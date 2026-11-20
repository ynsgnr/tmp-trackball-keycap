"""Keycap-mounted replacement base for the HG66 mini ring trackball.

Replaces the original finger-ring base with an open box that
  * holds the PCB and the 501020 battery (standing on edge beside the PCB),
  * takes the original top shell: at one end its hinge ledge sits in a slot
    between two tabs, at the other end the two original screws go in from below,
  * surrounds the shell's rim with an outer lip that covers the seam,
  * is fused onto an MX cross-stem low-profile 1u keycap (1u.stl).

Run:   python3 build.py
       -> trackball_base_plain.stl/.step   box only, flat underside at z = 0
          trackball_mount.stl/.step        box on the keycap
          preview.svg                      line drawing of the box
Print: 0.4 mm nozzle, 0.2 mm layers.  The plain base prints flat with no
       supports; the keycap version needs supports from the build plate.
       Cut off the three small pillars in the hinge slot after printing.

Axes (top view):  X = long axis,  +X = screw end,  -X = hinge end
                  Y = short axis, -Y = USB-C / power switch side, +Y = battery
The keycap sits under the -X/+Y corner, so the box only overhangs towards +X
and -Y (off the keyboard).  Mirror in the slicer for the opposite corner.

The wall follows the top shell's rim outline in outline.json, traced from the
underside photo and scan by fit_outline.py.  The PCB holes were measured on
the original base (ref/original_base.jpg) against its screw posts and checked
against the test fit (ref/pcb_test_fit.webp).  Numbers marked (*) are estimated
from photos; the rest was measured or set from test prints.
"""
import json
import tempfile
from functools import lru_cache

import cadquery as cq
import trimesh
from shapely.geometry import Polygon, box as rect

# ---- FDM tolerances (0.4 mm nozzle, 0.2 mm layers) -------------------------
CLR = 0.2          # clearance per mating face, horizontal and vertical
SAG = 0.2          # extra height under an overhang that has to stay clear
MIN_T = 0.8        # thinnest printed feature (two extrusion widths)

# ---- top shell --------------------------------------------------------------
RIM = Polygon(json.load(open("outline.json")))     # rim outline (fit_outline.py)
XH, YU, XS, YB = RIM.bounds     # hinge end, USB side, screw end, battery side
SCREW_DY   = 17.5               # distance between the two screw posts
SCREW_EDGE = 4.4                # screw centre to the +X end
SCREW_LEN  = 4.0                # screw length under the head
SCREW_GRIP = 2.6                # thread left to bite into the top-shell post
SCREW_HOLE = 1.6                # prints ~1.4-1.5: passes the thread, seats the 2.0 head
DRIVER_D   = 3.2                # counterbore for the head and screwdriver shaft
LEDGE_T    = 1.0                # shell's hinge ledge thickness

# ---- electronics ------------------------------------------------------------
PCB_T      = 1.0
PCB_FROM_BOTTOM = 6.0           # box underside up to the PCB standing plane (its underside)
PCB_FROM_TOP    = 7.0           # PCB standing plane up to the highest point of the lip
STAND_XY   = [(9.45, 7.2), (9.45, 5.0)]   # 1 x 1 mm PCB stands beside the screw bosses,
STAND_W    = 1.0                # mirrored to -y (clear of the underside side button)
PCB_UNDER  = 3.0                # tallest part under the PCB (side button)
PCB_SHIFT  = (-0.95, 0.80)      # correction to the first outline estimate (holes + test fit)
PCB_SHAPE  = [(x + PCB_SHIFT[0], y + PCB_SHIFT[1]) for x, y in
              [(-14.2, 6.3), (-13.4, 7.1), (11, 7.1), (11, 4.85), (14.8, 4.85), (14.8, -6.2),
               (11.2, -6.2), (11.2, -11.75), (-9.6, -11.75), (-14.2, -4.9)]]   # (*) outline
PCB_HOLES  = [(4.57, 5.66), (-7.96, 0.99)]          # locating holes, ~2.0 mm: measured on
                                # the original base (ref/original_base.jpg) against its screws,
                                # then moved 1 mm towards the hinge after a test fit, and the
                                # hinge-side one 1 mm towards the battery after another
USB_X      = (-10.0, -2.4)      # USB-C shell across X, measured on ref/pcb_test_fit.webp
USB_Y0     = -12.6              # USB-C mouth (it overhangs the PCB edge)
USB_BELOW  = 1.6                # how far the shell reaches below the PCB underside
PEG_D      = 1.0                # peg in those holes: 0.5 mm slack all round
BATTERY    = (20.0, 5.2, 9.0)   # length, width, height standing on edge; seen from
                                # above the pouch has fully rounded ends
BAT_X      = 0.0                # battery centre along X (where the bay is widest)

# ---- box --------------------------------------------------------------------
WALL       = 1.2                # wall inside the rim outline
FLOOR      = 1.6
SKIRT_OUT  = 1.2                # wall outside the rim outline, over the full height
SKIRT_H    = 1.0                # outer lip: stands LIP_H + this above the rim, all round
LIP_H      = 1.3                # wall top raised by this much in three stretches:
LIP_BOSS_X = 11.5               #   battery side, from the hinge gap to here (short of the boss)
LIP_END_Y  = 6.0                #   screw end, |y| below this (between the bosses)
HINGE_GAP  = 14.0               #   ...and not within this width at the hinge
WIN_X      = (-12.0, 11.5)      # USB-C + power switch opening along X
SILL_H     = 3.0                # wall left under that opening, from the box underside
BTN_Y      = (-1.5 + PCB_SHIFT[1], 5.5 + PCB_SHIFT[1])   # (*) underside-button access, +X wall

# ---- hinge (at the -X end, stacked in z) ------------------------------------
TAB_W      = 10.0               # width of both tabs
TAB_OUT    = 1.0                # both reach this far in from the wall's inner face
TAB_T      = 1.2                # lower tab, flush with the rim, sticking out of the wall
CLIP_TOP_T = 1.0                # upper tab, coming inwards out of the outer lip
SUPPORT_D  = 0.8                # cut-off print supports between the tabs

# ---- keycap (1u.stl: 18 x 18 x 4.8, flat top, 1 mm ceiling) ----------------
KEY, CAP_TOP, OVERLAP = 18.0, 4.8, 0.4

# ---- derived ----------------------------------------------------------------
Z0     = CAP_TOP - OVERLAP                  # underside of the floor
ZF     = Z0 + FLOOR                         # floor top
ZPCB   = Z0 + PCB_FROM_BOTTOM               # PCB underside (standing plane)
Z_TOP  = ZPCB + PCB_FROM_TOP                # highest point (hinge carrier on the lip)
SEAM   = Z_TOP - max(LIP_H + SKIRT_H, LEDGE_T + CLR + SAG + CLIP_TOP_T)   # rim plane
Z_SLOT = SEAM + LEDGE_T + CLR + SAG         # underside of the upper hinge tab
PCB_X0 = min(x for x, _ in PCB_SHAPE)       # PCB edge at the hinge end
PCB_Y1 = max(y for _, y in PCB_SHAPE)       # PCB edge on the battery side
KEY_XY = (XH - SKIRT_OUT + KEY / 2, YB + SKIRT_OUT - KEY / 2)   # keycap centre
ABOVE  = (SEAM - 1, Z_TOP + 1)              # z-range for cutters above the rim
OUTSIDE = -SKIRT_OUT - 2                    # "inset" well outside the part


def slab(x, y, z):
    """Axis-aligned block from (min, max) pairs."""
    return (cq.Workplane("XY").workplane(offset=z[0])
            .center(sum(x) / 2, sum(y) / 2).rect(x[1] - x[0], y[1] - y[0])
            .extrude(z[1] - z[0]))


def cyl(x, y, d, z):
    return cq.Workplane("XY").workplane(offset=z[0]).center(x, y).circle(d / 2).extrude(z[1] - z[0])


@lru_cache(maxsize=None)
def _outline_points(inset):
    ring = RIM.buffer(-inset, 32).exterior
    return tuple(ring.interpolate(i / 96, normalized=True).coords[0] for i in range(96))


def outline(inset, z):
    """Rim outline shrunk by `inset` (negative grows it), extruded over z = (min, max)."""
    return (cq.Workplane("XY").workplane(offset=z[0])
            .spline(_outline_points(inset), periodic=True).close().extrude(z[1] - z[0]))


def ring(inner, outer, z):
    """Band between two insets of the rim outline, e.g. ring(WALL, 0, z) is the wall."""
    return outline(outer, z).cut(outline(inner, (z[0] - 1, z[1] + 1)))


def shell():
    """Floor plus wall up to the rim plane."""
    return outline(-SKIRT_OUT, (Z0, SEAM)).cut(outline(WALL, (ZF, SEAM + 1)))


def screw_bosses():
    """Bosses at the +X end; screws enter from below through a deep counterbore."""
    x = XS - SCREW_EDGE
    seat = SEAM - (SCREW_LEN - SCREW_GRIP)        # underside of the screw head
    add, cut = cq.Workplane("XY"), cq.Workplane("XY")
    for y in (SCREW_DY / 2, -SCREW_DY / 2):
        add = add.union(cyl(x, y, DRIVER_D + 2 * MIN_T, (Z0, SEAM)))
        cut = cut.union(cyl(x, y, SCREW_HOLE, (Z0 - 1, SEAM + 1)))
        cut = cut.union(cyl(x, y, DRIVER_D, (Z0 - 1, seat)))
    return add.intersect(outline(0, (Z0, SEAM))), cut


def pcb_supports():
    """Rails under two PCB edges, a block under the USB-C shell (takes the push when a
    cable goes in; stops CLR under the shell, clear of its side legs), and a
    shouldered peg for each locating hole."""
    s = slab((1, 8.4), (PCB_Y1 - 1.0, PCB_Y1), (ZF, ZPCB))              # battery side
    for x, y in STAND_XY:
        for ys in (y, -y):                                               # mirrored stands
            s = s.union(slab((x - STAND_W / 2, x + STAND_W / 2), (ys - STAND_W / 2, ys + STAND_W / 2),
                             (ZF, ZPCB)))
    s = s.union(slab((USB_X[0] + 0.6, USB_X[1] - 0.6), (USB_Y0 + 0.2, USB_Y0 + 4.2),
                     (ZF, ZPCB - USB_BELOW - CLR)))                       # under the USB-C
    s = s.union(slab((PCB_X0 - 0.4, PCB_X0 + 1.0), (-3, 3), (ZF, ZPCB)))  # hinge end
    for x, y in PCB_HOLES:
        s = s.union(cyl(x, y, 3.2, (ZF, ZPCB))).union(cyl(x, y, PEG_D, (ZPCB, ZPCB + PCB_T + 0.4)))
    return s


def hinge():
    """Lower tab out of the wall (flush with the rim), a slot for the shell's hinge
    ledge, then an upper tab coming inwards out of the outer lip, which is raised
    locally to carry it.  Three thin pillars hold the upper tab up while printing."""
    y = (-TAB_W / 2 + CLR / 2, TAB_W / 2 - CLR / 2)
    x_in = XH + WALL + TAB_OUT
    here = slab((XH - SKIRT_OUT - 1, x_in), y, (SEAM - TAB_T, Z_SLOT + CLIP_TOP_T))
    lower = slab((XH + WALL - 0.1, x_in), y, (SEAM - TAB_T, SEAM))
    post = ring(-CLR, -SKIRT_OUT, (SEAM, Z_SLOT + CLIP_TOP_T)).intersect(here)
    upper = outline(-SKIRT_OUT, (Z_SLOT, Z_SLOT + CLIP_TOP_T)).intersect(here)
    clip = lower.union(post).union(upper)
    for ys in (y[0] + 0.9, 0, y[1] - 0.9):
        clip = clip.union(cyl(x_in - 0.7, ys, SUPPORT_D, (SEAM - 0.1, Z_SLOT + 0.1)))
    return clip


def lips():
    """Everything above the rim plane except the hinge: the wall top raised by LIP_H
    in three stretches, and the outer lip, which stands LIP_H + SKIRT_H above the rim
    all the way round, outside the shell."""
    raised = (slab((XH - 3, WIN_X[0]), (YU - 3, -HINGE_GAP / 2), ABOVE)          # USB-side corner
              .union(slab((XH - 3, LIP_BOSS_X), (HINGE_GAP / 2, YB + 3), ABOVE))  # battery side
              .union(slab((XS - 6, XS + 3), (-LIP_END_Y, LIP_END_Y), ABOVE)))     # screw end
    wall_top = ring(WALL, -SKIRT_OUT, (SEAM, SEAM + LIP_H)).intersect(raised)
    return wall_top.union(ring(-CLR, -SKIRT_OUT, (SEAM, SEAM + LIP_H + SKIRT_H)))


def openings():
    """USB-C/switch opening: the wall and lip removed over WIN_X above the sill, through
    their full thickness so the ends are clean faces.  Plus the side-button hole."""
    z = (Z0 + SILL_H, Z_TOP + 1)
    usb = slab(WIN_X, (YU - SKIRT_OUT - 1, YU + 6), z).intersect(ring(WALL, OUTSIDE, z))
    button = slab((XS - WALL - 1.0, XS + SKIRT_OUT + 1), BTN_Y, (ZPCB - PCB_UNDER - 0.4, ZPCB + 0.4))
    return usb.union(button)


def build():
    bosses, holes = screw_bosses()
    box = shell().union(bosses).union(pcb_supports()).union(hinge()).union(lips())
    return box.cut(holes).cut(openings())


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
    return cap.union(stem.cut(cross)).translate((*KEY_XY, 0))


def validate(box):
    """Fit and printability checks (each entry is True when it FAILS); returns True
    when all pass."""
    solid = box.val()

    def overlap(other):
        return solid.intersect(other.val()).Volume()

    bat_l, bat_w, bat_h = BATTERY
    room = RIM.buffer(-WALL - CLR)                       # usable floor plan
    pcb_plan = Polygon(PCB_SHAPE)
    r = bat_w / 2
    bat = rect(BAT_X - bat_l / 2 + r, PCB_Y1 + CLR + r, BAT_X + bat_l / 2 - r,
               PCB_Y1 + CLR + r + 1e-3).buffer(r, 32)            # top view: stadium
    pcb = (cq.Workplane("XY").workplane(offset=ZPCB + 0.01).polyline(PCB_SHAPE).close()
           .extrude(PCB_T - 0.01))
    pegs = len(PCB_HOLES) * 3.1416 * PEG_D ** 2 / 4 * (PCB_T - 0.01)
    shell_wall = ring(CLR, -CLR, (SEAM + 0.01, SEAM + LIP_H + SKIRT_H))   # where the shell sits
    hinge_zone = slab((XH - 1, XH + 3), (-TAB_W / 2, TAB_W / 2), (SEAM, Z_TOP + 1))
    failures = {
        "box is one solid":                  box.solids().size() != 1,
        "PCB outline fits inside the walls": not room.contains(pcb_plan),
        "PCB touches only its pegs":           overlap(pcb) - pegs > 0.01,
        "battery fits inside the walls":     not room.contains(bat),
        "battery touches nothing":
            overlap(cq.Workplane("XY").workplane(offset=ZF + 0.01)
                    .polyline(list(bat.exterior.coords)[:-1]).close().extrude(bat_h - 0.02)) > 1e-3,
        "room under the PCB for its parts":  ZPCB - ZF < PCB_UNDER + CLR - 1e-6,
        "PCB plane at PCB_FROM_BOTTOM / PCB_FROM_TOP":
            abs(ZPCB - Z0 - PCB_FROM_BOTTOM) + abs(solid.BoundingBox().zmax - ZPCB - PCB_FROM_TOP) > 1e-3,
        "PCB stands are under the PCB":
            not all(Polygon(PCB_SHAPE).contains(rect(x - STAND_W / 2, ys - STAND_W / 2,
                                                     x + STAND_W / 2, ys + STAND_W / 2))
                    for x, y in STAND_XY for ys in (y, -y)),
        "battery stays under the top shell": ZF + bat_h > SEAM + LIP_H - 0.1 + 1e-6,
        "outer lip leaves room for the shell wall":
            overlap(shell_wall.cut(hinge_zone).cut(ring(WALL, OUTSIDE, (SEAM, SEAM + LIP_H)))) > 1e-3,
        "nothing sticks up too far":           solid.BoundingBox().zmax > Z_TOP + 1e-6,
        "screw bosses inside the rim outline":
            not RIM.buffer(-MIN_T).contains(rect(XS - SCREW_EDGE - 0.1, -SCREW_DY / 2,
                                                 XS - SCREW_EDGE + 0.1, SCREW_DY / 2)),
        "no feature under two extrusions wide":
            min(WALL, SKIRT_OUT - CLR, PEG_D, SUPPORT_D, CLIP_TOP_T, TAB_T) < MIN_T - 1e-6,
    }
    for name, failed in failures.items():
        print(("FAIL  " if failed else "ok    ") + name)
    print(f"rim outline {XS - XH:.1f} x {YB - YU:.1f}; floor {Z0:.1f}-{ZF:.1f}, PCB seat {ZPCB:.1f}, "
          f"PCB top {ZPCB + PCB_T:.1f} ({ZPCB - ZF:.1f} free under it, {Z_TOP - ZPCB - PCB_T:.1f} below the top), rim {SEAM:.1f}, top {Z_TOP:.1f}; PCB-wall gap {room.exterior.distance(pcb_plan) + CLR:.2f}, "
          f"battery-wall gap {room.exterior.distance(bat) + CLR:.2f}")
    return not any(failures.values())


def export(box):
    with tempfile.NamedTemporaryFile(suffix=".stl") as tmp:
        cq.exporters.export(box, tmp.name, tolerance=0.01, angularTolerance=0.05)
        mesh = trimesh.load(tmp.name)
    cap = trimesh.load("1u.stl")
    centre = cap.bounds.mean(axis=0)
    cap.apply_translation([KEY_XY[0] - centre[0], KEY_XY[1] - centre[1], -cap.bounds[0][2]])
    mount = trimesh.boolean.union([cap, mesh], engine="manifold")
    plain = mesh.copy()
    plain.apply_translation([0, 0, -Z0])                 # flat underside at z = 0
    for name, part in (("trackball_mount", mount), ("trackball_base_plain", plain)):
        assert part.is_watertight and part.body_count == 1, name
        part.export(name + ".stl")
        print(name, "size", part.extents.round(1).tolist())
    # native CAD for SolidWorks etc.; the keycap there is a re-model of 1u.stl
    keycap = keycap_cad()
    print("keycap re-model volume %.1f vs 1u.stl %.1f mm3" % (keycap.val().Volume(), cap.volume))
    cq.exporters.export(box.translate((0, 0, -Z0)), "trackball_base_plain.step")
    cq.exporters.export(box.union(keycap), "trackball_mount.step")
    cq.exporters.export(box, "preview.svg", opt={"projectionDir": (0.5, -1.0, 2.0),
                                                 "showAxes": False, "width": 900, "height": 600})


if __name__ == "__main__":
    box = build()
    assert validate(box), "fit checks failed"
    export(box)
