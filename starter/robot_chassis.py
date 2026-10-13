"""
2WD robot chassis — single deck, 3D-printable.

    Uno R4 (+ custom hat) · 2 × TT gearmotors · 6×AA pack · printed ball castor
    · HC-SR04 ultrasonic mount · cable pass-throughs

Coordinate system: X = forward, Y = left, Z = up, origin on the ground under
the centre line. All dimensions in mm.

Printed parts (export each one separately):
    ChassisDeck        the main plate — print top face up, no supports
    MotorBracket(+1)   left motor bracket  ─┐ print lying on their side
    MotorBracket(-1)   right motor bracket ─┘ (L-profile flat on the bed)
    CastorBody         print upright
    CastorRing         print flat
Everything else in the assembly (motors, wheels, Arduino, battery, sensor,
ball) is a grey/coloured reference model so you can check the fit.

Hardware: M3 screws — 4× M3x30 + nuts (motors), 4× M3x10 + nuts (brackets),
4× M3x8 (Arduino, self-tapping into the bosses), 2× M3x12 (castor to deck),
2× M3x10 (castor ring), one 16 mm marble or steel ball, a velcro strap.

!! MEASURE BEFORE PRINTING !!
TT motor clones vary. Check the values marked "MEASURE" with calipers on
your motors and wheels. The motor holes are slotted (±2.5 mm) to absorb small
differences. Printing one MotorBracket first (≈15 min) is a cheap fit test.
"""

import math

from cadbuildr.foundation import (
    Assembly,
    Axis,
    Circle,
    Extrusion,
    Lathe,
    Line,
    Part,
    Point,
    Polygon,
    Rectangle,
    Sketch,
    show,
)

# ----------------------------------------------------------------------------
# Bought parts — MEASURE these
# ----------------------------------------------------------------------------
WHEEL_DIAMETER = 66.0        # MEASURE: kit TT wheels are 66 mm
WHEEL_WIDTH = 26.0           # MEASURE: 16 mm kit wheels, 26 mm for chunky yellow ones
WHEEL_GAP = 3.0              # gap between motor face and wheel

MOTOR_LENGTH = 70.0          # gearbox + motor can
MOTOR_HEIGHT = 22.5          # MEASURE: the face that stands vertical when mounted
MOTOR_THICKNESS = 18.8       # MEASURE: across the motor, along the shaft
SHAFT_FROM_END = 11.0        # MEASURE: shaft centre to the gearbox end of the motor
MOTOR_HOLES_FROM_SHAFT = (20.0, 37.5)  # MEASURE: the two through-holes, along the body
MOTOR_HOLE_SLOT = 2.5        # each hole is slotted ± this much along the body

ARDUINO_LENGTH, ARDUINO_WIDTH = 68.6, 53.3
# Uno / Uno R4 mounting holes (board coordinates, USB/power jack at x = 0)
ARDUINO_HOLES = [(14.0, 2.5), (15.3, 50.7), (66.1, 7.6), (66.1, 35.5)]

BATTERY_X, BATTERY_Y, BATTERY_Z = 47.0, 58.0, 31.0  # MEASURE your 6×AA holder

SR04_HOLE_SPACING = 26.0     # HC-SR04 transducer centres
SR04_HOLE_DIAMETER = 16.5    # transducers are Ø16, push-fit
SR04_HEIGHT = 20.0           # transducer centre above the deck top

BALL_DIAMETER = 16.0         # MEASURE: glass marble (16 mm) or steel ball

# ----------------------------------------------------------------------------
# Chassis design
# ----------------------------------------------------------------------------
DECK_X_MIN, DECK_X_MAX = -80.0, 95.0   # 175 mm long
DECK_HALF_WIDTH = 42.0                 # 84 mm wide; motors sit flush with the edge
DECK_THICKNESS = 4.0
DECK_CORNER = 6.0

AXLE_X = -45.0                          # drive wheels towards the rear
AXLE_Z = WHEEL_DIAMETER / 2
DECK_Z = AXLE_Z + MOTOR_HEIGHT / 2      # underside of the deck (= top of the motors)
TOP_Z = DECK_Z + DECK_THICKNESS

MOTOR_OUTER_Y = DECK_HALF_WIDTH
MOTOR_INNER_Y = MOTOR_OUTER_Y - MOTOR_THICKNESS

BRACKET_T = 4.0                         # bracket wall thickness
BRACKET_X0, BRACKET_X1 = AXLE_X + 8, AXLE_X + 48
BRACKET_FLANGE = 10.0                   # flange width under the deck
BRACKET_SCREW_X = (AXLE_X + 14, AXLE_X + 42)

M3_CLEAR = 3.4
M3_PILOT = 2.8                          # self-tapping M3 into plastic

ARDUINO_ORIGIN = (DECK_X_MIN + 2, -ARDUINO_WIDTH / 2)  # USB faces the rear edge
BOSS_HEIGHT, BOSS_DIAMETER = 6.0, 6.5

BATTERY_X0 = 25.0                       # battery holder rear edge
CRADLE_WALL, CRADLE_HEIGHT, CRADLE_CLEARANCE = 2.5, 6.0, 0.5
STRAP_SLOT = (20.0, 4.0)                # velcro strap slot (along X, along Y)

MOTOR_CABLE_HOLE = (17.0, 33.0, 8.0)    # x, ±y, diameter — just ahead of the motors
CENTRE_CABLE_SLOT = (12.0, 20.0, 10.0)  # x0, x1, width — reaches the underside

CASTOR_X = 81.0
CASTOR_OD = BALL_DIAMETER + 12
CASTOR_SCREW_Y = 10.0                   # deck → castor screws at (CASTOR_X, ±10)

SR04_PLATE_X0, SR04_PLATE_X1 = 92.0, DECK_X_MAX
SR04_PLATE_HALF_WIDTH, SR04_PLATE_HEIGHT = 27.0, 34.0

# Castor geometry (ball rests on a lip in the ring; ring screws to the body)
BALL_R = BALL_DIAMETER / 2
RING_HOLE_R = BALL_R - 0.75             # 0.75 mm lip holds the ball in
LIP_TOP_Z = BALL_R - math.sqrt(BALL_R**2 - RING_HOLE_R**2)
RING_Z0 = LIP_TOP_Z - 1.0               # 1 mm lip
RING_Z1 = RING_Z0 + 3.0
BORE_R = BALL_R + 0.4
BORE_TOP_Z = BALL_DIAMETER + 0.4
RING_SCREW_R = (BORE_R + CASTOR_OD / 2) / 2


# ----------------------------------------------------------------------------
# Small helpers (all coordinates are world coordinates)
# ----------------------------------------------------------------------------
def _xy_sketch(part, z):
    plane = part.xy() if z == 0 else part.xy().get_parallel_plane(z)
    return Sketch(plane)


def box(part, x0, y0, x1, y1, z0, z1, cut=False):
    s = _xy_sketch(part, z0)
    rect = Rectangle.from_2_points(Point(s, min(x0, x1), min(y0, y1)),
                                   Point(s, max(x0, x1), max(y0, y1)))
    part.add_operation(Extrusion([rect], z1 - z0, cut=cut))


def cylinder_z(part, x, y, r, z0, z1, cut=False):
    s = _xy_sketch(part, z0)
    part.add_operation(Extrusion([Circle(Point(s, x, y), r)], z1 - z0, cut=cut))


# Side-facing features (horizontal holes/cylinders).
#
# These sketch on the side planes *through the origin* and use the extrusion's
# start/end distance along the plane normal to place the feature. (Offsetting a
# side plane with get_parallel_plane() is avoided on purpose: for the rotated
# xz/yz planes it applies the plane rotation twice and puts the sketch in the
# wrong place.) The plane is always picked so its normal points away from the
# origin, towards the feature, and every cut sits at the far end of its part.
#
# Plane mappings, (u, v) sketch coordinates -> world, and the normal:
#   yz: (y, z), +X      zy: (z, y), -X      zx: (z, x), +Y      xz: (x, z), -Y
def _side_extrude(part, plane, shapes_fn, near, far, cut):
    s = Sketch(plane)
    part.add_operation(Extrusion(shapes_fn(s), far, start=near, cut=cut))


def _split(a, b):
    lo, hi = min(a, b), max(a, b)
    if lo < 0 < hi:
        return [(lo, 0.0), (0.0, hi)]
    return [(lo, hi)]


def cylinder_y(part, x, z, r, y0, y1, cut=False):
    """Cylinder along Y between y0 and y1."""
    for lo, hi in _split(y0, y1):
        if lo >= 0:
            _side_extrude(part, part.zx(), lambda s: [Circle(Point(s, z, x), r)], lo, hi, cut)
        else:
            _side_extrude(part, part.xz(), lambda s: [Circle(Point(s, x, z), r)], -hi, -lo, cut)


def slot_y(part, x, z, r, half_len, y0, y1):
    """Slot elongated along X, cut through a wall that is thin in Y."""
    for lo, hi in _split(y0, y1):
        if lo >= 0:
            plane, uv = part.zx(), (lambda px, pz: (pz, px))
        else:
            plane, uv = part.xz(), (lambda px, pz: (px, pz))
            lo, hi = -hi, -lo

        (ax, ay), (bx, by) = uv(x - half_len, z - r), uv(x + half_len, z + r)
        corners = ((min(ax, bx), min(ay, by)), (max(ax, bx), max(ay, by)))
        ends = [uv(x + dx, z) for dx in (-half_len, half_len)]

        # rectangle + two round ends, each cut separately
        _side_extrude(part, plane, lambda s: [Rectangle.from_2_points(
            Point(s, *corners[0]), Point(s, *corners[1]))], lo, hi, True)
        for end in ends:
            _side_extrude(part, plane, lambda s, e=end: [Circle(Point(s, *e), r)], lo, hi, True)


def cylinder_x(part, y, z, r, x0, x1, cut=False):
    """Cylinder along X between x0 and x1."""
    for lo, hi in _split(x0, x1):
        if lo >= 0:
            _side_extrude(part, part.yz(), lambda s: [Circle(Point(s, y, z), r)], lo, hi, cut)
        else:
            _side_extrude(part, part.zy(), lambda s: [Circle(Point(s, z, y), r)], -hi, -lo, cut)


def rounded_plate(part, x0, y0, x1, y1, r, z0, z1):
    box(part, x0 + r, y0, x1 - r, y1, z0, z1)
    box(part, x0, y0 + r, x1, y1 - r, z0, z1)
    for cx in (x0 + r, x1 - r):
        for cy in (y0 + r, y1 - r):
            cylinder_z(part, cx, cy, r, z0, z1)


# ----------------------------------------------------------------------------
# Printed parts
# ----------------------------------------------------------------------------
class ChassisDeck(Part):
    def __init__(self):
        super().__init__()
        rounded_plate(self, DECK_X_MIN, -DECK_HALF_WIDTH, DECK_X_MAX, DECK_HALF_WIDTH,
                      DECK_CORNER, DECK_Z, TOP_Z)
        self.arduino_bosses()
        self.battery_cradle()
        self.ultrasonic_mount()
        self.holes()
        self.paint("purple")

    def arduino_bosses(self):
        ox, oy = ARDUINO_ORIGIN
        for hx, hy in ARDUINO_HOLES:
            x, y = ox + hx, oy + hy
            cylinder_z(self, x, y, BOSS_DIAMETER / 2, TOP_Z, TOP_Z + BOSS_HEIGHT)
            # blind pilot hole — stops 0.6 mm above the underside
            cylinder_z(self, x, y, M3_PILOT / 2, DECK_Z + 0.6, TOP_Z + BOSS_HEIGHT, cut=True)

    def battery_cradle(self):
        x0 = BATTERY_X0 - CRADLE_CLEARANCE
        x1 = BATTERY_X0 + BATTERY_X + CRADLE_CLEARANCE
        y1 = BATTERY_Y / 2 + CRADLE_CLEARANCE
        z1 = TOP_Z + CRADLE_HEIGHT
        w = CRADLE_WALL
        # rear and front walls, side walls with a gap for the strap / wires
        box(self, x0 - w, -y1 - w, x0, y1 + w, TOP_Z, z1)
        box(self, x1, -y1 - w, x1 + w, y1 + w, TOP_Z, z1)
        mid = (x0 + x1) / 2
        for sign in (1, -1):
            for xa, xb in ((x0 - w, mid - 14), (mid + 14, x1 + w)):
                box(self, xa, sign * y1, xb, sign * (y1 + w), TOP_Z, z1)
        # strap slots just outside the side walls
        sx, sy = STRAP_SLOT
        ys = y1 + w + 1.5
        for sign in (1, -1):
            box(self, mid - sx / 2, sign * ys, mid + sx / 2, sign * (ys + sy),
                DECK_Z - 1, TOP_Z + 1, cut=True)

    def ultrasonic_mount(self):
        hw = SR04_PLATE_HALF_WIDTH
        box(self, SR04_PLATE_X0, -hw, SR04_PLATE_X1, hw, TOP_Z, TOP_Z + SR04_PLATE_HEIGHT)
        for sign in (1, -1):   # stiffening ribs behind the plate
            box(self, SR04_PLATE_X0 - 6, sign * (hw - 3.5), SR04_PLATE_X0,
                sign * (hw - 0.5), TOP_Z, TOP_Z + 10)
        for sign in (1, -1):
            cylinder_x(self, sign * SR04_HOLE_SPACING / 2, TOP_Z + SR04_HEIGHT,
                       SR04_HOLE_DIAMETER / 2, SR04_PLATE_X0 - 1, SR04_PLATE_X1 + 1, cut=True)

    def holes(self):
        z0, z1 = DECK_Z - 1, TOP_Z + 1
        # motor bracket screws
        for x in BRACKET_SCREW_X:
            for sign in (1, -1):
                cylinder_z(self, x, sign * _bracket_screw_y(), M3_CLEAR / 2, z0, z1, cut=True)
        # castor screws
        for sign in (1, -1):
            cylinder_z(self, CASTOR_X, sign * CASTOR_SCREW_Y, M3_CLEAR / 2, z0, z1, cut=True)
        # cable pass-throughs: one beside each motor's terminals + one centre slot
        cx, cy, cd = MOTOR_CABLE_HOLE
        for sign in (1, -1):
            cylinder_z(self, cx, sign * cy, cd / 2, z0, z1, cut=True)
        sx0, sx1, sw = CENTRE_CABLE_SLOT
        box(self, sx0, -sw / 2, sx1, sw / 2, z0, z1, cut=True)


def _bracket_screw_y():
    return MOTOR_INNER_Y - BRACKET_T - BRACKET_FLANGE / 2


class MotorBracket(Part):
    """L-bracket: flange screws under the deck, the leg holds the motor.

    side = +1 for the left motor (+Y), -1 for the right motor.
    """

    def __init__(self, side: int = 1):
        super().__init__()
        s = side
        leg_y0, leg_y1 = s * (MOTOR_INNER_Y - BRACKET_T), s * MOTOR_INNER_Y
        flange_y0 = s * (MOTOR_INNER_Y - BRACKET_T - BRACKET_FLANGE)
        # leg (vertical, beside the motor) and flange (horizontal, under the deck)
        box(self, BRACKET_X0, leg_y0, BRACKET_X1, leg_y1, DECK_Z - MOTOR_HEIGHT, DECK_Z)
        box(self, BRACKET_X0, flange_y0, BRACKET_X1, leg_y0, DECK_Z - BRACKET_T, DECK_Z)
        # gusset in the middle of the L
        gx = (BRACKET_X0 + BRACKET_X1) / 2
        box(self, gx - 2, flange_y0, gx + 2, leg_y0, DECK_Z - 12, DECK_Z)
        # deck screws through the flange
        for x in BRACKET_SCREW_X:
            cylinder_z(self, x, s * _bracket_screw_y(), M3_CLEAR / 2,
                       DECK_Z - BRACKET_T - 1, DECK_Z + 1, cut=True)
        # slotted motor holes through the leg
        for d in MOTOR_HOLES_FROM_SHAFT:
            slot_y(self, AXLE_X + d, AXLE_Z, M3_CLEAR / 2, MOTOR_HOLE_SLOT,
                   leg_y0 - s * 1, leg_y1 + s * 1)
        self.paint("purple")


class CastorBody(Part):
    """Column from the deck down to the ball; the ball sits in a bore at the bottom."""

    def __init__(self):
        super().__init__()
        r = CASTOR_OD / 2
        cylinder_z(self, CASTOR_X, 0, r, RING_Z1, DECK_Z)
        cylinder_z(self, CASTOR_X, 0, BORE_R, RING_Z1 - 1, BORE_TOP_Z, cut=True)
        # lighten the column
        cylinder_z(self, CASTOR_X, 0, 6.0, BORE_TOP_Z + 3, DECK_Z + 1, cut=True)
        # pilot holes: deck screws (top) and ring screws (bottom)
        for sign in (1, -1):
            cylinder_z(self, CASTOR_X, sign * CASTOR_SCREW_Y, M3_PILOT / 2,
                       DECK_Z - 12, DECK_Z + 1, cut=True)
            cylinder_z(self, CASTOR_X + sign * RING_SCREW_R, 0, M3_PILOT / 2,
                       RING_Z1 - 1, RING_Z1 + 10, cut=True)
        self.paint("purple")


class CastorRing(Part):
    """Retaining ring: a 1 mm lip under the ball, counterbored above it."""

    def __init__(self):
        super().__init__()
        r = CASTOR_OD / 2
        cylinder_z(self, CASTOR_X, 0, r, RING_Z0, RING_Z1)
        cylinder_z(self, CASTOR_X, 0, RING_HOLE_R, RING_Z0 - 1, RING_Z1 + 1, cut=True)
        cylinder_z(self, CASTOR_X, 0, BORE_R, LIP_TOP_Z, RING_Z1 + 1, cut=True)
        for sign in (1, -1):
            cylinder_z(self, CASTOR_X + sign * RING_SCREW_R, 0, M3_CLEAR / 2,
                       RING_Z0 - 1, RING_Z1 + 1, cut=True)
        self.paint("purple")


# ----------------------------------------------------------------------------
# Reference models (not printed) — for checking the fit in the viewer
# ----------------------------------------------------------------------------
class RefMotor(Part):
    def __init__(self, side: int = 1):
        super().__init__()
        y0, y1 = side * MOTOR_INNER_Y, side * MOTOR_OUTER_Y
        box(self, AXLE_X - SHAFT_FROM_END, y0, AXLE_X - SHAFT_FROM_END + MOTOR_LENGTH, y1,
            DECK_Z - MOTOR_HEIGHT, DECK_Z)
        self.paint("yellow")


class RefWheel(Part):
    def __init__(self, side: int = 1):
        super().__init__()
        y0 = side * (MOTOR_OUTER_Y + WHEEL_GAP)
        y1 = side * (MOTOR_OUTER_Y + WHEEL_GAP + WHEEL_WIDTH)
        cylinder_y(self, AXLE_X, AXLE_Z, WHEEL_DIAMETER / 2, y0, y1)
        self.paint("black")


class RefArduino(Part):
    def __init__(self):
        super().__init__()
        ox, oy = ARDUINO_ORIGIN
        z = TOP_Z + BOSS_HEIGHT
        box(self, ox, oy, ox + ARDUINO_LENGTH, oy + ARDUINO_WIDTH, z, z + 1.6)
        box(self, ox + 2, oy + 2, ox + ARDUINO_LENGTH - 2, oy + ARDUINO_WIDTH - 2,
            z + 1.6, z + 14)   # custom hat (stand-in)
        self.paint("dark_cyan", transparency=0.3)


class RefBattery(Part):
    def __init__(self):
        super().__init__()
        box(self, BATTERY_X0, -BATTERY_Y / 2, BATTERY_X0 + BATTERY_X, BATTERY_Y / 2,
            TOP_Z, TOP_Z + BATTERY_Z)
        self.paint("dark_grey")


class RefUltrasonic(Part):
    def __init__(self):
        super().__init__()
        z = TOP_Z + SR04_HEIGHT
        box(self, SR04_PLATE_X0 - 1.6, -22.5, SR04_PLATE_X0, 22.5, z - 10, z + 10)
        for sign in (1, -1):
            cylinder_x(self, sign * SR04_HOLE_SPACING / 2, z, 8.0,
                       SR04_PLATE_X0, SR04_PLATE_X0 + 12)
        self.paint("blue")


class RefBall(Part):
    def __init__(self):
        super().__init__()
        s = Sketch(self.xz())   # (u, v) -> (x, z), through y = 0
        n = 24
        pts = [Point(s, CASTOR_X + BALL_R * math.sin(math.pi * i / n),
                     BALL_R - BALL_R * math.cos(math.pi * i / n)) for i in range(n + 1)]
        lines = [Line(pts[i], pts[i + 1]) for i in range(n)] + [Line(pts[n], pts[0])]
        axis = Axis(Line(Point(s, CASTOR_X, 0), Point(s, CASTOR_X, BALL_DIAMETER)))
        self.add_operation(Lathe(Polygon(lines), axis))
        self.paint("white")


class RobotChassis(Assembly):
    def __init__(self, show_reference: bool = True):
        super().__init__()
        for part in (ChassisDeck(), MotorBracket(1), MotorBracket(-1),
                     CastorBody(), CastorRing()):
            self.add_component(part)
        if show_reference:
            for part in (RefMotor(1), RefMotor(-1), RefWheel(1), RefWheel(-1),
                         RefArduino(), RefBattery(), RefUltrasonic(), RefBall()):
                self.add_component(part)


if __name__ == "__main__":
    # Full robot with reference parts. To export a single printed part, e.g.:
    #   show(ChassisDeck())   show(MotorBracket(1))   show(CastorBody())
    show(RobotChassis())