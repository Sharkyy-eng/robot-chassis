"""
MEBOT robot base — a 3D-printable take on the Makeblock mBot base.

  * Top plate 120 x 100 mm (mBot: 114 x 90), walls 31 mm high
  * Front (+X): motor lobes, front wall with "MEBOT" engraved 1 mm (replaces
    the mBot smile)
  * Back (-X): sensor arms, trapezoid wall and castor tab
  * MEBOT swoosh "M" cut through both side walls and the back wall
  * Drop-in Arduino Uno mount pushed to the front (corner ridges + ledges, no
    screws), leaving the back half of the top plate free for the battery pack
  * Every mBot hole pattern kept at its original spacing (Makeblock 8/16 mm
    grid); holes that are tapped on the real mBot are 3.3 mm self-tapping
    pilots for M4 screws

Coordinates: X forward (motor-lobe end), Y left, Z up. mm.
Outline and hole data come from mebot_base_data.py (made by make_base_data.py
from the mBot STEP file). This file only needs cadbuildr-foundation.

Printing: the sensor arms rise 10 mm above the top plate, so print it right
side up with supports under the top plate (or ask for arms trimmed flush to
print it upside-down without supports).
"""

from cadbuildr.foundation import (
    Extrusion,
    Line,
    Part,
    Point,
    Polygon,
    Rectangle,
    Sketch,
    show,
)

try:
    from starter import mebot_base_data as D      # inside the starter-template layout
except ImportError:
    import mebot_base_data as D                    # running next to the data file

# ----------------------------------------------------------------------------
# Main dimensions
# ----------------------------------------------------------------------------
LENGTH, WIDTH = 120.0, 100.0
WALL = 2.5                    # printed wall / plate thickness (mBot sheet is 2.0)
TOP_Z = 30.0                  # top surface of the top plate

# Arduino Uno drop-in mount (board turned 90°: its long side runs across the robot,
# so the ridges stand on solid plate instead of over the two long slots)
UNO_LONG, UNO_SHORT, UNO_PCB = 68.6, 53.3, 1.6
UNO_FRONT_GAP = 1.0           # gap between the front ridges and the front wall
UNO_CLEARANCE = 0.4           # gap around the board; 0.3 = snugger
LEDGE_HEIGHT = 3.0            # board sits this high (clears solder pins)
LEDGE_WIDTH = 1.5             # how far the ledge reaches under the board edge
RIDGE_T = 2.0                 # ridge wall thickness
RIDGE_LEG = 10.0              # length of each corner ridge leg
RIDGE_ABOVE_PCB = 1.2         # ridge top above the board's top surface
CONNECTOR_SIDE = -1           # USB + DC jack face the right side (-Y); +1 for left
ENGRAVE_DEPTH = 1.0           # "MEBOT" on the front wall
JACK_STOP = (18.0, 24.0)      # stop block on the connector edge, between jack and USB
                              # (board y-coordinates: jack ≈ 3-12 mm, USB ≈ 32-44 mm)


# ----------------------------------------------------------------------------
# Sketch helpers
# ----------------------------------------------------------------------------
def _poly(sketch, pts):
    p = [Point(sketch, u, v) for u, v in pts]
    return Polygon([Line(p[i], p[(i + 1) % len(p)]) for i in range(len(p))])


def _xy(part, z):
    return Sketch(part.xy() if z == 0 else part.xy().get_parallel_plane(z))


def flat(part, ring_xy, z0, z1, cut=False):
    """Outline in (X, Y), extruded vertically from z0 to z1."""
    part.add_operation(Extrusion([_poly(_xy(part, z0), ring_xy)], z1 - z0, cut=cut))


def box(part, x0, y0, x1, y1, z0, z1, cut=False):
    s = _xy(part, z0)
    r = Rectangle.from_2_points(Point(s, min(x0, x1), min(y0, y1)),
                                Point(s, max(x0, x1), max(y0, y1)))
    part.add_operation(Extrusion([r], z1 - z0, cut=cut))


# Side-facing panels are sketched on side planes through the origin; the
# extrusion's start/end place them along the normal. (Planes: yz -> (Y, Z),
# normal +X; zy -> (Z, Y), -X; zx -> (Z, X), +Y; xz -> (X, Z), -Y.)
def wall_xz(part, ring_xz, y_near, y_far, side, cut=False):
    """Panel in (X, Z) — a side wall — between |Y| = y_near and y_far."""
    if side > 0:
        plane, pts = part.zx(), [(z, x) for x, z in ring_xz]
    else:
        plane, pts = part.xz(), ring_xz
    part.add_operation(Extrusion([_poly(Sketch(plane), pts)], y_far, start=y_near, cut=cut))


def wall_yz(part, ring_yz, x_near, x_far, side, cut=False):
    """Panel in (Y, Z) — an end wall — between |X| = x_near and x_far."""
    if side > 0:
        plane, pts = part.yz(), ring_yz
    else:
        plane, pts = part.zy(), [(z, y) for y, z in ring_yz]
    part.add_operation(Extrusion([_poly(Sketch(plane), pts)], x_far, start=x_near, cut=cut))


# ----------------------------------------------------------------------------
class MebotBase(Part):
    def __init__(self):
        super().__init__()
        hx, hy = LENGTH / 2, WIDTH / 2
        # top plate
        box(self, -hx, -hy, hx, hy, TOP_Z - WALL, TOP_Z)
        # side walls (outline is the same on both sides; M is mirrored so it
        # reads correctly from the outside of each wall)
        for side, m_cuts in ((1, D.SIDE["m_left"]), (-1, D.SIDE["m_right"])):
            wall_xz(self, D.SIDE["outline"], hy - WALL, hy, side)
            for ring in D.SIDE["holes"] + m_cuts:
                wall_xz(self, ring, hy - WALL - 1, hy + 1, side, cut=True)
        # end walls: front (engraved MEBOT) and back (swoosh M)
        for side, data, cuts in ((1, D.FRONT_WALL, []), (-1, D.BACK_WALL, D.BACK_WALL["m"])):
            wall_yz(self, data["outline"], hx - WALL, hx, side)
            for ring in data["holes"] + cuts:
                wall_yz(self, ring, hx - WALL - 1, hx + 1, side, cut=True)
        # engrave the letters into the front face, then put the letter
        # counters (the middles of B and O) back
        for letter in D.FRONT_WALL["engrave"]:
            wall_yz(self, letter["outline"], hx - ENGRAVE_DEPTH, hx + 1, 1, cut=True)
            for counter in letter["counters"]:
                wall_yz(self, counter, hx - ENGRAVE_DEPTH, hx, 1)
        # bottom tabs (castor tab at the front, 6-hole tab at the back)
        for tab in D.BOTTOM_TABS:
            flat(self, tab["outline"], 0, WALL)
            for ring in tab["holes"]:
                flat(self, ring, -1, WALL + 1, cut=True)
        # Arduino mount, then the top-plate holes (cut last, so they also
        # pass cleanly through anything standing on the plate)
        self.uno_mount()
        for ring in D.TOP_HOLES:
            flat(self, ring, TOP_Z - WALL - 1, TOP_Z + 1, cut=True)
        self.paint("purple")

    def uno_mount(self):
        # board footprint (long side along Y), pushed up to the front wall
        bx1 = LENGTH / 2 - WALL - UNO_FRONT_GAP - RIDGE_T - UNO_CLEARANCE
        bx0 = bx1 - UNO_SHORT
        by = UNO_LONG / 2
        c, t, leg = UNO_CLEARANCE, RIDGE_T, RIDGE_LEG
        z0 = TOP_Z
        z_ledge = z0 + LEDGE_HEIGHT
        z_top = z_ledge + UNO_PCB + RIDGE_ABOVE_PCB
        for sy in (1, -1):
            for x_edge, sx in ((bx0, -1), (bx1, 1)):
                # ridge along the board's long edge, at this corner
                xa = x_edge + sx * c
                box(self, xa, sy * (by - leg), xa + sx * t, sy * (by + c + t), z0, z_top)
                # ridge along the short edge (skipped on the connector side)
                if sy != CONNECTOR_SIDE:
                    box(self, x_edge - sx * (leg - c), sy * (by + c),
                        x_edge + sx * (c + t), sy * (by + c + t), z0, z_top)
                # support ledge under the long edge (clear of the header pins)
                box(self, x_edge, sy * (by - leg + 2), x_edge - sx * LEDGE_WIDTH,
                    sy * by, z0, z_ledge)
        # stop block on the connector edge, between DC jack and USB.
        # Component side up with the connectors facing -Y, the board's own
        # y-axis (jack at y≈3-12, USB at y≈32-44) points towards -X.
        s0, s1 = JACK_STOP
        ys = CONNECTOR_SIDE * (by + c)
        if CONNECTOR_SIDE < 0:
            xa, xb = bx1 - s1, bx1 - s0
        else:
            xa, xb = bx0 + s0, bx0 + s1
        box(self, xa, ys, xb, ys + CONNECTOR_SIDE * t, z0, z_top)


if __name__ == "__main__":
    show(MebotBase())
