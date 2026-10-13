"""
Builds mebot_base_data.py — the panel outlines and hole patterns for the
MEBOT robot base, taken from the mBot base STEP file and resized to a
120 x 100 mm top plate, with the MEBOT swoosh "M" in place of the mBot "M".

    pip install cadquery-ocp shapely scikit-image pillow numpy
    python make_base_data.py mBot_Base.stp starter/mebot_logo.png

Only needed if you change the STEP file, the logo or the resize amounts below.
mebot_base.py itself only needs cadbuildr-foundation.

How the resize works: panels get longer/wider, but every hole pattern keeps its
original spacing (so Makeblock parts on the 8 mm / 16 mm grid still fit).
  * Length 114 -> 120: everything in the front half moves +3 mm, rear half -3 mm.
  * Width   90 -> 100: everything on the left moves +5 mm, right -5 mm.
"""

import sys

import numpy as np
from PIL import Image
from shapely.affinity import rotate, scale, translate
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import nearest_points, polygonize, unary_union
from skimage import measure

LENGTH_GROW = (120.0 - 114.0) / 2   # per end
WIDTH_GROW = (100.0 - 90.0) / 2     # per side
WALL = 2.5                          # printed wall thickness (mBot sheet is 2.0)
M_MARGIN = 2.0                      # min material between the M and other holes/edges
M_MIN_CUT = 1.0                     # thinner parts of the M cut-out are dropped
M_BRIDGE = 1.4                      # bridge that holds the M's inner triangle in place
SIMPLIFY = 0.03
TAPPED_STEP_D = (3.1, 3.35)         # the STEP draws tapped holes at ~3.2 mm
SELF_TAP_D = 3.3                    # M4 self-tapping pilot (use 2.5 for M3)
TEXT = "MEBOT"                      # engraved on the front wall instead of the smile
TEXT_HEIGHT = 9.0                   # letter height
TEXT_Z = (9.5, 18.5)                # band between the wall's hole rows


# ----------------------------------------------------------------------------
# STEP -> 2D sections
# ----------------------------------------------------------------------------
def load_sections(step_path):
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection
    from OCP.gp import gp_Dir, gp_Pln, gp_Pnt
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_EDGE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    reader = STEPControl_Reader()
    reader.ReadFile(step_path)
    reader.TransferRoots()
    shape = reader.OneShape()

    def section(p, n):
        sec = BRepAlgoAPI_Section(shape, gp_Pln(gp_Pnt(*p), gp_Dir(*n)))
        sec.Build()
        out, ex = [], TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
        while ex.More():
            c = BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
            d = GCPnts_QuasiUniformDeflection(c, 0.05)
            out.append([(d.Value(i).X(), d.Value(i).Y(), d.Value(i).Z())
                        for i in range(1, d.NbPoints() + 1)])
            ex.Next()
        return out

    # STEP axes: x = width, y = up, z = length (smile wall at z ~ +2.5)
    return {
        "side": section((44, 0, 0), (1, 0, 0)),
        "top": section((0, 29, 0), (0, 1, 0)),
        "bottom": section((0, 1, 0), (0, 1, 0)),
        "rear": section((0, 0, 2.5), (0, 0, 1)),
        "front": section((0, 0, -109.5), (0, 0, 1)),
    }


def world(p):
    """STEP (x, y, z) -> robot (X forward, Y left, Z up), centred on the plate.

    Front (+X) is the end with the motor lobes (the mBot smile wall)."""
    return (p[2] + 53.5, -p[0], p[1])


def polygons(segs, axes):
    lines = [LineString([tuple(round(world(p)[a], 2) for a in axes) for p in s])
             for s in segs if len(s) >= 2]
    return sorted(polygonize(unary_union(lines)), key=lambda p: -p.area)


# ----------------------------------------------------------------------------
# MEBOT "M"
# ----------------------------------------------------------------------------
def trace_mebot_m(logo_path):
    img = np.array(Image.open(logo_path).convert("RGBA")).astype(int)
    r, g, b, a = (img[..., i] for i in range(4))
    red = (a > 128) & (r > 120) & (r > g * 1.8) & (r > b * 1.8) & (g < 120)
    red |= (a > 128) & (r > 60) & (r > g * 2.2) & (r > b * 2.2)  # fold shading
    geom = MultiPolygon()
    for c in measure.find_contours(np.pad(red.astype(float), 2), 0.5):
        if len(c) >= 4:
            geom = geom.symmetric_difference(
                Polygon([(x - 2, -(y - 2)) for y, x in c]).buffer(0))
    geom = geom.buffer(1.5).buffer(-1.5)
    m = max(getattr(geom, "geoms", [geom]), key=lambda p: p.area)
    x0, y0, x1, y1 = m.bounds
    return translate(m, -(x0 + x1) / 2, -(y0 + y1) / 2)   # centred on its bounding box


def place_m(m, region, near, mirror_u, max_size, lock_u=False):
    """Largest M (up to max_size) that fits in region, close to `near`."""
    if mirror_u:
        m = scale(m, -1, 1, origin=(0, 0))
    hull = m.convex_hull
    w = max(m.bounds[2] - m.bounds[0], m.bounds[3] - m.bounds[1])
    best = (0, None)
    for dx in ([0.0] if lock_u else np.linspace(-6, 6, 25)):
        for dz in np.linspace(-5, 5, 21):
            cx, cz = near[0] + dx, near[1] + dz
            lo, hi = 0.0, max_size / w
            for _ in range(22):
                mid = (lo + hi) / 2
                if region.contains(translate(scale(hull, mid, mid, origin=(0, 0)), cx, cz)):
                    lo = mid
                else:
                    hi = mid
            if lo > best[0] + 1e-6:
                best = (lo, (cx, cz))
    k, (cx, cz) = best
    cut = translate(scale(m, k, k, origin=(0, 0)), cx, cz)
    cut = cut.buffer(-M_MIN_CUT / 2).buffer(M_MIN_CUT / 2)   # drop un-printable tips
    # bridges: tie every enclosed island back to the surrounding wall
    polys = list(getattr(cut, "geoms", [cut]))
    out = []
    for p in polys:
        for ring in p.interiors:
            a, bpt = nearest_points(Polygon(ring).exterior, p.exterior)
            v = np.array([bpt.x - a.x, bpt.y - a.y])
            v = v / (np.linalg.norm(v) + 1e-9)
            bridge = LineString([(a.x - v[0], a.y - v[1]), (bpt.x + v[0], bpt.y + v[1])])
            p = p.difference(bridge.buffer(M_BRIDGE / 2, cap_style=2))
        out.extend(getattr(p, "geoms", [p]))
    out = [q for q in out if q.area > M_MIN_CUT ** 2]
    assert all(len(q.interiors) == 0 for q in out), "M still has a floating island"
    print(f"  M fitted: {k * w:.1f} mm, centre ({cx:.1f}, {cz:.1f}), {len(out)} cut piece(s)")
    return out


# ----------------------------------------------------------------------------
def shift_by_sign(geom, axis, amount):
    """Move a whole loop +amount if its centroid is positive on `axis`, else -amount."""
    c = geom.centroid.x if axis == 0 else geom.centroid.y
    d = amount if c >= 0 else -amount
    return translate(geom, d if axis == 0 else 0, d if axis == 1 else 0)


def stretch(poly, axis, amount, threshold=0.0):
    """Move outline vertices beyond +/-threshold outwards by amount (along axis)."""
    def mv(p):
        p = list(p)
        if abs(p[axis]) > threshold:
            p[axis] += amount if p[axis] > 0 else -amount
        return tuple(p)
    return Polygon([mv(p) for p in poly.exterior.coords])


def classify(poly):
    """Describe a hole: round hole (diameter), slot, or other shape."""
    x0, y0, x1, y1 = poly.bounds
    w, h = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    if abs(w - h) < 0.15 and poly.area / (np.pi * (w / 2) ** 2) > 0.93:
        return dict(kind="hole", c=(round(cx, 2), round(cy, 2)), d=round((w + h) / 2, 2))
    mrr = poly.minimum_rotated_rectangle
    pts = np.array(mrr.exterior.coords)[:4]
    e1, e2 = pts[1] - pts[0], pts[2] - pts[1]
    l1, l2 = np.linalg.norm(e1), np.linalg.norm(e2)
    long_, short = max(l1, l2), min(l1, l2)
    axis = e1 if l1 >= l2 else e2
    ideal = short * (long_ - short) + np.pi * short ** 2 / 4
    if long_ / short > 1.4 and abs(poly.area - ideal) < 0.04 * ideal:
        ang = float(np.degrees(np.arctan2(axis[1], axis[0]))) % 180
        c = poly.centroid
        return dict(kind="slot", c=(round(c.x, 2), round(c.y, 2)), length=round(float(long_), 2),
                    width=round(float(short), 2), angle=round(ang, 1))
    return dict(kind="shape", c=(round(cx, 2), round(cy, 2)), size=(round(w, 2), round(h, 2)))


def tidy_holes(polys, tag):
    """Round holes become exact circles; tapped ones become self-tapping pilots."""
    rings, table = [], []
    for p in polys:
        info = classify(p)
        if info["kind"] == "hole":
            d = info["d"]
            if TAPPED_STEP_D[0] <= d <= TAPPED_STEP_D[1]:
                info.update(d=SELF_TAP_D, note="tapped on mBot -> M4 self-tap pilot")
            else:
                info["d"] = round(d * 2) / 2 if abs(d * 2 - round(d * 2)) < 0.12 else d
            p = Point(*info["c"]).buffer(info["d"] / 2, 48)
        info["panel"] = tag
        table.append(info)
        rings.append(ring(p))
    return rings, table


def text_shapes(text, height, centre):
    from matplotlib.font_manager import FontProperties
    from matplotlib.textpath import TextPath
    tp = TextPath((0, 0), text, size=10, prop=FontProperties(family="DejaVu Sans", weight="bold"))
    g = MultiPolygon()
    for r in tp.to_polygons():
        if len(r) >= 3:
            g = g.symmetric_difference(Polygon(r).buffer(0))
    x0, y0, x1, y1 = g.bounds
    k = height / (y1 - y0)
    g = scale(translate(g, -(x0 + x1) / 2, -(y0 + y1) / 2), k, k, origin=(0, 0))
    g = translate(g, *centre)
    return [p for p in getattr(g, "geoms", [g])]


def ring(geom):
    g = geom.simplify(SIMPLIFY)
    return [(round(x, 3), round(y, 3)) for x, y in list(g.exterior.coords)[:-1]]


def main(step_path, logo_path):
    S = load_sections(step_path)
    m_shape = trace_mebot_m(logo_path)
    table = []

    # --- side walls (X, Z) ---------------------------------------------------
    side = polygons(S["side"], (0, 2))[0]
    holes = [Polygon(h) for h in side.interiors]
    old_m = max((h for h in holes if 25 < h.centroid.x < 50), key=lambda h: h.area)
    holes = [shift_by_sign(h, 0, LENGTH_GROW) for h in holes if h is not old_m]
    outline = stretch(side, 0, LENGTH_GROW)
    region = Polygon(outline.exterior).difference(
        unary_union([h.buffer(M_MARGIN) for h in holes])).buffer(-M_MARGIN)
    near = (old_m.centroid.x + LENGTH_GROW, old_m.centroid.y)
    print("side walls:")
    # Read from outside: on the left (+Y) wall "rightwards" is -X, so mirror.
    m_left = place_m(m_shape, region, near, mirror_u=True, max_size=24.0)
    m_right = place_m(m_shape, region, near, mirror_u=False, max_size=24.0)
    side_rings, t = tidy_holes(holes, "side wall (X, Z)")
    table += t
    side_data = dict(outline=ring(outline), holes=side_rings,
                     m_left=[ring(q) for q in m_left], m_right=[ring(q) for q in m_right])

    # --- top plate (X, Y): holes keep their positions ----------------------
    top = polygons(S["top"], (0, 1))[0]
    top_rings, t = tidy_holes([Polygon(h) for h in top.interiors], "top plate (X, Y)")
    table += t

    # --- end walls (Y, Z) ----------------------------------------------------
    # end walls/tabs reach 0.5 mm into the side walls so the print is one solid
    widen = WIDTH_GROW + 1.5
    # front = mBot smile wall: smile removed, MEBOT engraved instead
    fw = polygons(S["rear"], (1, 2))[0]
    fw_holes = [Polygon(h) for h in fw.interiors]
    smile = max(fw_holes, key=lambda h: h.area)
    fw_rings, t = tidy_holes([h for h in fw_holes if h is not smile], "front wall (Y, Z)")
    table += t
    letters = text_shapes(TEXT, TEXT_HEIGHT, (0.0, sum(TEXT_Z) / 2))  # read from the front: +Y is right
    front_data = dict(outline=ring(stretch(fw, 0, widen, threshold=30)), holes=fw_rings,
                      engrave=[dict(outline=ring(p), counters=[[(round(x, 3), round(y, 3)) for x, y in list(r.coords)[:-1]] for r in p.interiors]) for p in letters])

    # back = mBot M wall: trapezoid, MEBOT swoosh M
    bw = polygons(S["front"], (1, 2))[0]
    bw_holes = [Polygon(h) for h in bw.interiors]
    old_bm = max(bw_holes, key=lambda h: h.area)
    bw_holes = [h for h in bw_holes if h is not old_bm]
    bw_outline = stretch(bw, 0, widen, threshold=30)
    bw_region = Polygon(bw_outline.exterior).difference(
        unary_union([h.buffer(M_MARGIN) for h in bw_holes] or [Point(999, 999)])).buffer(-M_MARGIN)
    print("back wall:")
    # Seen from behind (looking towards +X) "rightwards" is -Y, so mirror.
    b_m = place_m(m_shape, bw_region, (0.0, old_bm.centroid.y),
                  mirror_u=True, max_size=26.0, lock_u=True)
    bw_rings, t = tidy_holes(bw_holes, "back wall (Y, Z)")
    table += t
    back_data = dict(outline=ring(bw_outline), holes=bw_rings, m=[ring(q) for q in b_m])

    # --- bottom tabs (X, Y) ----------------------------------------------
    tabs = []
    for p in polygons(S["bottom"], (0, 1)):
        x0, y0, x1, y1 = p.bounds
        if min(x1 - x0, y1 - y0) < 5 or p.area < 300:
            continue   # skip the side-wall slivers and hole discs
        p = shift_by_sign(p, 0, LENGTH_GROW)
        name = "front tab (X, Y)" if p.centroid.x > 0 else "castor tab, back (X, Y)"
        p_out = stretch(p, 1, widen, threshold=30)
        rings, t = tidy_holes([Polygon(h) for h in p.interiors], name)
        table += t
        tabs.append(dict(name=name, outline=ring(p_out), holes=rings))

    with open("mebot_base_data.py", "w") as f:
        f.write('"""Generated by make_base_data.py from the mBot base STEP file.\n\n'
                'Coordinates in mm. X forward (front = motor-lobe end), Y left, Z up.\n'
                'Side walls: (X, Z). Top plate and bottom tabs: (X, Y). End walls: (Y, Z).\n'
                'Each ring is a closed outline (last point joins the first)."""\n\n')
        for name, val in [("SIDE", side_data), ("TOP_HOLES", top_rings),
                          ("FRONT_WALL", front_data), ("BACK_WALL", back_data),
                          ("BOTTOM_TABS", tabs), ("HOLE_TABLE", table)]:
            f.write(f"{name} = {val!r}\n\n")
    kinds = {}
    for h in table:
        kinds[h["kind"]] = kinds.get(h["kind"], 0) + 1
    print("wrote mebot_base_data.py:", kinds,
          "| self-tap pilots:", sum(1 for h in table if "note" in h))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "starter/mebot_logo.png")
