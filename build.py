"""Keycap-mounted replacement base for the HG66 mini ring trackball.

Replaces the original finger-ring base with an open box that
  * holds the PCB and the 501020 battery (standing on edge beside the PCB),
  * takes the original top shell: a hinge tab sticks out of the end wall at one
    end, two original screws go in from below at the other end,
  * is fused onto an MX cross-stem low-profile 1u keycap (1u.stl).

Run:   python3 build.py   ->  trackball_mount.stl (with keycap),
       trackball_base_plain.stl (box only), preview.svg   (units: mm)
Print: upright (stem down), 0.4 mm nozzle / 0.2 mm layers, supports from the
       build plate only.

Axes (top view):  X = long axis,  +X = screw end,  -X = hinge end
                  Y = short axis, -Y = USB-C / power switch side, +Y = battery
The keycap sits under the -X/+Y corner, so the box only overhangs towards +X
and -Y (off the keyboard).  Mirror in the slicer for the opposite corner.

Shell/PCB numbers marked (*) are estimated from photos; the rest was measured.
"""
import cadquery as cq
import trimesh

# ---- FDM tolerances (0.4 mm nozzle, 0.2 mm layers) -------------------------
CLR      = 0.2    # clearance per mating face, horizontal and vertical
HOLE_CLR = 0.4    # added to the diameter of vertical holes (they print small)
# Printed features are kept at >= 0.8 mm (two extrusion widths) and all levels
# (floor, PCB seat, rim, lip, tab) land on 0.2 mm layers.

# ---- top shell --------------------------------------------------------------
L, W, R    = 37.0, 30.0, 9.0   # (*) rim outline and corner radius
TOP_WALL   = 1.0               # (*) top-shell wall thickness at the rim
SCREW_DY   = 17.5              # (*) distance between the two screw posts
SCREW_EDGE = 4.4               # (*) screw centre to the +X end
SCREW_LEN  = 4.0               # screw length under the head
SCREW_HEAD = 2.0               # screw head diameter
SCREW_GRIP = 2.6               # thread left to bite into the top-shell post
DRIVER_D   = 3.2               # counterbore: head + screwdriver shaft
TAB_W      = 10.0              # hinge tab width (original base)
TAB_OUT    = 1.0               # how far the tab sticks out of the wall, inwards
TAB_T      = 1.2               # (*) tab thickness below the rim plane

# ---- electronics ------------------------------------------------------------
PCB_T      = 1.0
PCB_DROP   = 5.8               # (*) rim down to the PCB top face
PCB_UNDER  = 3.0               # tallest part under the PCB (side button)
PCB_X      = (-14.2, 14.8)     # (*) PCB extent along X
PCB_Y      = (-11.75, 7.1)     # (*) PCB extent along Y
PCB_HOLES  = [(5.6, 4.85), (-6.1, -0.8)]   # (*) locating holes, ~1.7 mm
PEG_D      = 1.2               # loose on purpose: hole positions are +-0.5
BATTERY    = (20.0, 5.2, 10.0) # length, width, height standing on edge
WIN_X      = (-12.0, 11.5)     # (*) USB-C + power switch opening along X
BTN_Y      = (-1.5, 5.5)       # (*) access to the underside button, +X wall

# ---- box --------------------------------------------------------------------
WALL, FLOOR = 1.6, 1.6
LIP_H, LIP_T = 1.2, 0.8        # locating lip inside the top shell
LIP_X      = (-8.0, 8.0)

# ---- keycap (1u.stl: 18 x 18 x 4.8, flat top, 1 mm ceiling) ----------------
KEY, CAP_TOP, OVERLAP = 18.0, 4.8, 0.4

# ---- derived levels ---------------------------------------------------------
Z0   = CAP_TOP - OVERLAP                 # underside of the floor
ZF   = Z0 + FLOOR                        # floor top
ZPCB = ZF + PCB_UNDER + CLR              # PCB underside
SEAM = ZPCB + PCB_T + PCB_DROP           # rim; the top shell sits here
XH, XS = -L / 2, L / 2                   # hinge end, screw end


def slab(x, y, z):
    """Axis-aligned block from (min, max) pairs."""
    return (cq.Workplane("XY").workplane(offset=z[0])
            .center(sum(x) / 2, sum(y) / 2).rect(x[1] - x[0], y[1] - y[0])
            .extrude(z[1] - z[0]))


def cyl(x, y, d, z):
    return cq.Workplane("XY").workplane(offset=z[0]).center(x, y).circle(d / 2).extrude(z[1] - z[0])


def outline(inset, z):
    return (cq.Workplane("XY").workplane(offset=z[0]).rect(L - 2 * inset, W - 2 * inset)
            .extrude(z[1] - z[0]).edges("|Z").fillet(R - inset))


def shell():
    return outline(0, (Z0, SEAM)).cut(outline(WALL, (ZF, SEAM + 1)))


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


def lip():
    """Rib on the battery-side wall that rises inside the top shell."""
    yo = W / 2 - TOP_WALL - CLR - 0.1      # 0.1 extra: shell wall is an estimate
    yi, yw = yo - LIP_T, W / 2 - WALL + 0.2
    prof = [(yw, SEAM - 2.0 - (yw - yi)), (yi, SEAM - 2.0), (yi, SEAM + LIP_H),
            (yo, SEAM + LIP_H), (yo, SEAM), (yw, SEAM)]       # 45 deg underside
    return (cq.Workplane("YZ").polyline(prof).close().extrude(LIP_X[1] - LIP_X[0])
            .translate((LIP_X[0], 0, 0)))


def hinge_tab():
    """Ledge sticking out of the -X end wall into the box, flush with the rim.
    It is free underneath (a 1 mm overhang, printable without support)."""
    x = (XH + WALL - 0.1, XH + WALL + TAB_OUT)
    y = (-TAB_W / 2 + CLR / 2, TAB_W / 2 - CLR / 2)
    return slab(x, y, (SEAM - TAB_T, SEAM))


def usb_opening():
    """Removes the -Y wall over WIN_X through its full thickness, so the wall ends
    are clean full-height faces even where the wall starts to curve."""
    wall_only = slab((XH - 1, XS + 1), (-W / 2 - 1, -W / 2 + R), (ZF, SEAM + 1)).cut(
        outline(WALL, (ZF - 1, SEAM + 2)))
    return slab(WIN_X, (-W / 2 - 1, -W / 2 + R), (ZF, SEAM + 1)).intersect(wall_only)


def button_opening():
    return slab((XS - WALL - 0.5, XS + 1), BTN_Y, (ZF, ZPCB + 0.4))


def build():
    bosses, holes = screw_bosses()
    box = shell().cut(usb_opening()).union(bosses).union(pcb_supports()).union(lip()).union(hinge_tab())
    return box.cut(holes).cut(button_opening())


def validate(box):
    """Check that the PCB, battery and top-shell rim have room, and print key gaps."""
    solid = box.val()
    def overlap(x, y, z):
        return solid.intersect(slab(x, y, z).val()).Volume()
    bat_l, bat_w, bat_h = BATTERY
    lip_inner = W / 2 - TOP_WALL - CLR - 0.1 - LIP_T
    checks = {
        "PCB body collides (pegs excluded)":
            overlap((PCB_X[0], 11), PCB_Y, (ZPCB + 0.01, ZPCB + PCB_T))
            - len(PCB_HOLES) * 3.1416 * PEG_D ** 2 / 4 * (PCB_T - 0.01) > 0.01,
        "under-PCB clearance < 3.2":            ZPCB - ZF < PCB_UNDER + CLR - 1e-6,
        "battery collides":
            overlap((-bat_l / 2, bat_l / 2), (PCB_Y[1] + CLR, PCB_Y[1] + CLR + bat_w),
                    (ZF + 0.01, ZF + bat_h)) > 1e-3,
        "battery bay narrower than battery+0.4": lip_inner - PCB_Y[1] < bat_w + 2 * CLR,
        "battery taller than cavity":            SEAM - ZF < bat_h - 1e-6,
        "top-shell rim seat obstructed":
            overlap((XH, XS), (-W / 2, W / 2), (SEAM + 0.01, SEAM + 3))
            - overlap((XH + TOP_WALL + CLR, XS - TOP_WALL - CLR),
                      (-W / 2 + TOP_WALL + CLR, W / 2 - TOP_WALL - CLR),
                      (SEAM + 0.01, SEAM + 3)) > 1e-3,
        "screw head has no seat":                SCREW_HEAD - (SCREW_HEAD - HOLE_CLR) < 0.3,
    }
    for name, failed in checks.items():
        print(("FAIL  " if failed else "ok    ") + name)
    print(f"levels: floor {Z0:.1f}-{ZF:.1f}, PCB seat {ZPCB:.1f}, rim {SEAM:.1f}; "
          f"battery bay {lip_inner - PCB_Y[1]:.1f} wide, {SEAM - ZF:.1f} deep")
    return not any(checks.values())


if __name__ == "__main__":
    box = build()
    ok = validate(box)
    cq.exporters.export(box, "/tmp/_box.stl", tolerance=0.01, angularTolerance=0.05)
    cq.exporters.export(box, "preview.svg", opt={"projectionDir": (0.5, -1.0, 2.0),
                                                 "showAxes": False, "width": 900, "height": 600})
    cap = trimesh.load("1u.stl")
    c = cap.bounds.mean(axis=0)
    cap.apply_translation([XH + KEY / 2 - c[0], W / 2 - KEY / 2 - c[1], -cap.bounds[0][2]])
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
