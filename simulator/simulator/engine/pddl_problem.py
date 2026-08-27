"""PDDL problem-file editing for mission planning.

A plan folder's `problem.pddl` is a *template*: a runnable problem in its own
right (fixed source/destination), plus a waypoint graph (locations, safe and
restricted routes, no-fly zones) built around that specific source and
destination. Picking a new start/destination on the map cannot just move
those two points and leave the waypoints where they were - a waypoint graph
built for a mission near Chennai is not a sensible detour for a mission near
Warangal, and "shortest path via a waypoint on the other side of the planet"
is correctly unsolvable.

Instead the *whole* graph is relocated: every other location is carried along
by the same rotate+scale+translate that maps the template's original
source->destination segment onto the newly picked one, so the detour's shape
(how far each waypoint sits off the direct line, in what direction) is
preserved wherever the mission is retargeted. lat/lon is treated as locally
flat (an equirectangular projection centred on the source of each frame),
which is accurate at the city/regional scale these detours operate at.

Distance/energy-required is recomputed for every edge from the moved
coordinates, at that edge's own original energy-per-metre rate (a plan author
can and does make some edges pricier than others - e.g. this template's
direct, restricted route costs about twice as much per metre as its safe
detour - so a single global rate would misprice the detour after a move).

This works entirely by text substitution over the flat `(= (attr obj) value)`
/ `(connected A B)` facts already in the file - no PDDL parser, and nothing
here is specific to any one plan's location names, so it works for any future
plan folder that shares the same `latitude`/`longitude`/`distance`/
`energy-required`/`connected` vocabulary (guaranteed by sharing `domain.pddl`).
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

EARTH_RADIUS_M = 6_371_000.0
METRES_PER_DEG_LAT = 111_320.0
DEFAULT_ENERGY_RATE = 0.0125  # energy units per metre, if an edge has no reference cost

_LATLON_FACT = re.compile(
    r"\(=\s*\((latitude|longitude)\s+(\S+)\)\s*([-\d.eE]+)\s*\)"
)
_CONNECTED_FACT = re.compile(r"\(connected\s+(\S+)\s+(\S+)\)")


@dataclass(frozen=True)
class LatLon:
    lat: float
    lon: float


def haversine_m(a: LatLon, b: LatLon) -> float:
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)
    dlat = lat2 - lat1
    dlon = math.radians(b.lon - a.lon)
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def read_locations(problem_text: str) -> dict[str, LatLon]:
    """Every `(= (latitude X) v)` / `(= (longitude X) v)` fact, merged by X."""
    lats: dict[str, float] = {}
    lons: dict[str, float] = {}
    for kind, name, value in _LATLON_FACT.findall(problem_text):
        target = lats if kind == "latitude" else lons
        target[name] = float(value)
    return {
        name: LatLon(lat=lats[name], lon=lons[name])
        for name in lats.keys() & lons.keys()
    }


def _read_edges(problem_text: str) -> list[tuple[str, str]]:
    return _CONNECTED_FACT.findall(problem_text)


def _fact_value(problem_text: str, predicate: str, a: str, b: str) -> float | None:
    pattern = re.compile(
        rf"\(=\s*\({re.escape(predicate)}\s+{re.escape(a)}\s+{re.escape(b)}\)\s*([-\d.eE]+)\s*\)"
    )
    match = pattern.search(problem_text)
    return float(match.group(1)) if match else None


def _set_fact_value(problem_text: str, predicate: str, a: str, b: str, value: float) -> str:
    pattern = re.compile(
        rf"(\(=\s*\({re.escape(predicate)}\s+{re.escape(a)}\s+{re.escape(b)}\)\s*)"
        rf"[-\d.eE]+(\s*\))"
    )

    def _repl(match: re.Match) -> str:
        return f"{match.group(1)}{value:.2f}{match.group(2)}"

    return pattern.sub(_repl, problem_text, count=1)


def _set_latlon(problem_text: str, name: str, point: LatLon) -> str:
    for kind, value in (("latitude", point.lat), ("longitude", point.lon)):
        pattern = re.compile(
            rf"(\(=\s*\({kind}\s+{re.escape(name)}\)\s*)[-\d.eE]+(\s*\))"
        )
        problem_text = pattern.sub(
            lambda m, v=value: f"{m.group(1)}{v:.7f}{m.group(2)}", problem_text, count=1
        )
    return problem_text


# ---- Local-planar projection, for the rotate+scale+translate ----


def _metres_per_deg_lon(lat_deg: float) -> float:
    return METRES_PER_DEG_LAT * math.cos(math.radians(lat_deg))


def _to_local(point: LatLon, origin: LatLon) -> tuple[float, float]:
    """(east_m, north_m) of `point` relative to `origin`."""
    x = (point.lon - origin.lon) * _metres_per_deg_lon(origin.lat)
    y = (point.lat - origin.lat) * METRES_PER_DEG_LAT
    return x, y


def _from_local(offset: tuple[float, float], origin: LatLon) -> LatLon:
    x, y = offset
    return LatLon(
        lat=origin.lat + y / METRES_PER_DEG_LAT,
        lon=origin.lon + x / _metres_per_deg_lon(origin.lat),
    )


def _similarity_transform(old_source: LatLon, old_dest: LatLon, new_source: LatLon, new_dest: LatLon):
    """A function mapping any `LatLon` through the rotate+scale+translate that
    takes `old_source -> new_source` and `old_dest -> new_dest`."""
    old_vec = _to_local(old_dest, old_source)
    old_len = math.hypot(*old_vec)
    if old_len < 1.0:
        # Degenerate template (source == destination) - translate only.
        def translate_only(point: LatLon) -> LatLon:
            return _from_local(_to_local(point, old_source), new_source)

        return translate_only

    new_vec = _to_local(new_dest, new_source)
    scale = math.hypot(*new_vec) / old_len
    angle = math.atan2(new_vec[1], new_vec[0]) - math.atan2(old_vec[1], old_vec[0])
    cos_a, sin_a = math.cos(angle), math.sin(angle)

    def transform(point: LatLon) -> LatLon:
        x, y = _to_local(point, old_source)
        rotated = (x * cos_a - y * sin_a, x * sin_a + y * cos_a)
        scaled = (rotated[0] * scale, rotated[1] * scale)
        return _from_local(scaled, new_source)

    return transform


# ---- Segment/polygon intersection, for the restricted-area check ----

_Point = tuple[float, float]


def _convex_hull(points: list[_Point]) -> list[_Point]:
    """Andrew's monotone chain, counter-clockwise, duplicates removed.

    The 4 corners of a restricted area are clicked in whatever order feels
    natural to the user (e.g. top-left, top-right, bottom-left, bottom-right
    - not a walk around the boundary), which as a raw polygon is
    self-intersecting: a bowtie, not a rectangle. Every geometry check below
    assumes a simple polygon, so the hull - the one unambiguous simple
    polygon those 4 points define - is used instead of the raw click order.
    """
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o: _Point, a: _Point, b: _Point) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[_Point] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[_Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _orientation(a: _Point, b: _Point, c: _Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: _Point, b: _Point, p: _Point) -> bool:
    return min(a[0], b[0]) - 1e-9 <= p[0] <= max(a[0], b[0]) + 1e-9 and \
        min(a[1], b[1]) - 1e-9 <= p[1] <= max(a[1], b[1]) + 1e-9


def _segments_intersect(p1: _Point, p2: _Point, p3: _Point, p4: _Point) -> bool:
    d1, d2 = _orientation(p3, p4, p1), _orientation(p3, p4, p2)
    d3, d4 = _orientation(p1, p2, p3), _orientation(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 != 0 and d2 != 0 and d3 != 0 and d4 != 0:
        return True
    # Collinear/touching cases: only count them if the point genuinely lies
    # on the other segment's span, not just on its infinite line.
    if d1 == 0 and _on_segment(p3, p4, p1):
        return True
    if d2 == 0 and _on_segment(p3, p4, p2):
        return True
    if d3 == 0 and _on_segment(p1, p2, p3):
        return True
    if d4 == 0 and _on_segment(p1, p2, p4):
        return True
    return False


def _point_in_polygon(point: _Point, polygon: list[_Point]) -> bool:
    """Standard ray-casting point-in-polygon test."""
    x, y = point
    inside = False
    n = len(polygon)
    x2, y2 = polygon[-1]
    for x1, y1 in polygon:
        if (y1 > y) != (y2 > y):
            x_at_y = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_at_y:
                inside = not inside
        x2, y2 = x1, y1
    return inside


def _segment_crosses_polygon(seg_a: _Point, seg_b: _Point, polygon: list[_Point]) -> bool:
    """Whether the segment `seg_a`->`seg_b` passes through `polygon` at all -
    crossing an edge, or running entirely inside it."""
    if len(polygon) < 3:
        return False
    n = len(polygon)
    for i in range(n):
        if _segments_intersect(seg_a, seg_b, polygon[i], polygon[(i + 1) % n]):
            return True
    midpoint = ((seg_a[0] + seg_b[0]) / 2.0, (seg_a[1] + seg_b[1]) / 2.0)
    return _point_in_polygon(midpoint, polygon)


def _points_close(p1: _Point, p2: _Point, eps: float = 0.5) -> bool:
    return abs(p1[0] - p2[0]) < eps and abs(p1[1] - p2[1]) < eps


def _segment_blocked_by_polygon(a: _Point, b: _Point, polygon: list[_Point]) -> bool:
    """Whether the segment `a`->`b` cuts through `polygon`'s interior.

    Unlike `_segment_crosses_polygon`, this tolerates `a`/`b` themselves
    being polygon vertices (or lying on a polygon edge) - needed for the
    visibility graph below, where the candidate endpoints usually *are*
    polygon corners and merely touching one is not a real obstruction.
    """
    n = len(polygon)
    a_idx = next((i for i, p in enumerate(polygon) if _points_close(a, p)), None)
    b_idx = next((i for i, p in enumerate(polygon) if _points_close(b, p)), None)
    if a_idx is not None and b_idx is not None:
        # Both endpoints are (convex) polygon vertices: adjacent ones are a
        # real boundary edge (not blocked); anything else is a diagonal,
        # which for a convex polygon necessarily cuts through the interior.
        # Decided by exact adjacency rather than the general test below,
        # because its own fallback - "is the midpoint inside?" - is exactly
        # the ill-posed question for a point that sits precisely *on* the
        # boundary, which an edge's own midpoint always does.
        diff = abs(a_idx - b_idx)
        return diff != 1 and diff != n - 1

    for i in range(n):
        p, q = polygon[i], polygon[(i + 1) % n]
        a_shared = _points_close(a, p) or _points_close(a, q)
        b_shared = _points_close(b, p) or _points_close(b, q)
        if a_shared and b_shared:
            continue  # (a, b) is this polygon edge itself - travelling along the boundary is fine
        if _segments_intersect(a, b, p, q) and not (a_shared or b_shared):
            return True
    midpoint = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    return _point_in_polygon(midpoint, polygon)


def _shortest_detour(source: _Point, destination: _Point, polygon: list[_Point]) -> list[_Point]:
    """Shortest path from `source` to `destination` that does not cut through
    `polygon`, via a visibility graph over {source, destination} + the
    polygon's own vertices - the classic construction for routing around
    polygonal obstacles, and exact (not a heuristic) for this few a corners.

    Returns the full path including both endpoints; `path[1:-1]` are the
    via-points to insert between them.
    """
    nodes = [source, destination] + list(polygon)
    n = len(nodes)
    edge_len: list[list[float]] = [[math.inf] * n for _ in range(n)]
    for i in range(n):
        edge_len[i][i] = 0.0
        for j in range(i + 1, n):
            if not _segment_blocked_by_polygon(nodes[i], nodes[j], polygon):
                d = math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
                edge_len[i][j] = edge_len[j][i] = d

    # Dijkstra from node 0 (source) to node 1 (destination); n <= 6 in
    # practice, so the plain O(n^2) scan is more than fast enough.
    best = [math.inf] * n
    best[0] = 0.0
    prev = [-1] * n
    visited = [False] * n
    for _ in range(n):
        u = min((i for i in range(n) if not visited[i]), key=lambda i: best[i], default=None)
        if u is None or best[u] == math.inf:
            break
        visited[u] = True
        for v in range(n):
            if not visited[v] and edge_len[u][v] < math.inf:
                candidate = best[u] + edge_len[u][v]
                if candidate < best[v]:
                    best[v] = candidate
                    prev[v] = u

    if best[1] == math.inf:
        return [source, destination]  # no path found (degenerate polygon) - fall back to direct

    path_idx = []
    cur = 1
    while cur != -1:
        path_idx.append(cur)
        cur = prev[cur]
    path_idx.reverse()
    return [nodes[i] for i in path_idx]


def _find_matching_paren(text: str, open_pos: int) -> int:
    depth = 0
    for i in range(open_pos, len(text)):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return i
    raise ValueError("unbalanced parentheses")


def _section_span(text: str, keyword: str) -> tuple[int, int]:
    """(open, close) character indices of a `(:keyword ...)` block."""
    match = re.search(rf"\(:{re.escape(keyword)}\b", text)
    if not match:
        raise ValueError(f"no (:{keyword} ...) section found")
    open_pos = match.start()
    return open_pos, _find_matching_paren(text, open_pos)


def _add_locations(problem_text: str, names: list[str]) -> str:
    """Declare new `- location` objects in the `(:objects ...)` block."""
    _, close_pos = _section_span(problem_text, "objects")
    addition = "".join(f"\n        {name} - location" for name in names)
    return problem_text[:close_pos] + addition + "\n    " + problem_text[close_pos:]


def _add_init_facts(problem_text: str, facts_text: str) -> str:
    """Append raw fact text into the `(:init ...)` block."""
    _, close_pos = _section_span(problem_text, "init")
    return problem_text[:close_pos] + facts_text + "\n    " + problem_text[close_pos:]


def _set_detour_route(
    problem_text: str,
    source_name: str,
    destination_name: str,
    new_source: LatLon,
    new_destination: LatLon,
    via_points: list[LatLon],
) -> str:
    """Make `source -> via_points... -> destination` the *only* legal route:
    strip every existing `safe-route` fact, declare the via-points as new
    location objects, and connect the chain with fresh
    `connected`/`safe-route`/`distance`/`energy-required` facts."""
    result = _strip_safe_routes(problem_text)
    if not via_points:
        return result  # nothing to chain through - caller keeps this a dead end deliberately

    names = [f"detour{i + 1}" for i in range(len(via_points))]
    result = _add_locations(result, names)

    chain_names = [source_name] + names + [destination_name]
    chain_coords = [new_source] + via_points + [new_destination]

    facts: list[str] = []
    for name, coord in zip(names, via_points):
        facts.append(f"\n        (= (latitude {name}) {coord.lat:.7f})")
        facts.append(f"\n        (= (longitude {name}) {coord.lon:.7f})")
    for a_name, a_pt, b_name, b_pt in zip(chain_names, chain_coords, chain_names[1:], chain_coords[1:]):
        distance_m = haversine_m(a_pt, b_pt)
        energy = distance_m * DEFAULT_ENERGY_RATE
        for x, y in ((a_name, b_name), (b_name, a_name)):
            facts.append(f"\n        (connected {x} {y})")
            facts.append(f"\n        (safe-route {x} {y})")
            facts.append(f"\n        (= (distance {x} {y}) {distance_m:.2f})")
            facts.append(f"\n        (= (energy-required {x} {y}) {energy:.2f})")

    return _add_init_facts(result, "".join(facts))


def _strip_safe_routes(problem_text: str) -> str:
    """Remove every existing `(safe-route A B)` fact and its line."""
    return re.sub(r"[ \t]*\(safe-route\s+\S+\s+\S+\)\n?", "", problem_text)


def _set_direct_route_only(problem_text: str, source_name: str, destination_name: str) -> str:
    """Make the direct leg the *only* legal route: strip every detour
    `safe-route` fact and add just `source<->destination`.

    ENHSP's default search is satisficing, not optimal - if both the direct
    leg and the detour were left legal, it could still return either one.
    Removing the alternative instead of merely permitting the direct leg
    makes the outcome deterministic.
    """
    stripped = _strip_safe_routes(problem_text)
    addition = (
        f"\n        (safe-route {source_name} {destination_name})"
        f"\n        (safe-route {destination_name} {source_name})\n"
    )
    matches = list(_CONNECTED_FACT.finditer(stripped))
    if not matches:
        return stripped + addition
    insert_at = matches[-1].end()
    return stripped[:insert_at] + addition + stripped[insert_at:]


def retarget_problem(
    problem_text: str,
    new_source: LatLon,
    new_destination: LatLon,
    *,
    no_fly_zone: list[LatLon] | None = None,
    source_name: str = "source",
    destination_name: str = "destination",
) -> str:
    """Relocate the whole waypoint graph to a newly picked source/destination.

    Every location is carried through the same rotate+scale+translate that
    maps the template's original source->destination segment onto the new
    one, preserving the detour's shape. Every edge's distance/energy-required
    is then recomputed from the moved coordinates, at that edge's own
    original energy-per-metre rate.

    Whether the mission is safe-routed at all is an explicit input, not
    something inferred from the template: `no_fly_zone` is `None`/empty
    unless the caller actually marked one, as a polygon (in the order its
    corners were picked - typically 4 points). Auto-detecting a restriction
    from where the template's own `(no-fly-zone W)` waypoint happened to
    land after the transform does not work - that point is carried along by
    the *same* proportional rotate+scale+translate as everything else, so
    its distance from the new line necessarily scales down with a short hop
    and up with a long one, making "blocked" depend on how far apart the two
    picked points are rather than on any real obstruction.

    - `no_fly_zone` given (>= 3 points) and the direct source->destination
      line actually crosses it (an edge, or running through its interior):
      the detour `safe-route` facts are left as the template wrote them; the
      direct leg stays illegal.
    - otherwise (no polygon, one with too few points, or one the direct line
      does not actually cross): every `safe-route` fact is replaced with
      just `source<->destination` - the detour is not merely more expensive,
      it does not exist as an option.

    Only one route is ever left legal, so ENHSP's (satisficing, not optimal)
    search has nothing to choose between and the outcome is deterministic.
    """
    locations = read_locations(problem_text)
    if source_name not in locations or destination_name not in locations:
        raise ValueError(
            f"template has no latitude/longitude for {source_name!r}/{destination_name!r}"
        )

    transform = _similarity_transform(
        locations[source_name], locations[destination_name], new_source, new_destination
    )
    moved = {name: transform(point) for name, point in locations.items()}
    # Pin the endpoints exactly to the picked points, rather than trusting
    # floating-point round-trip through the transform to land on them.
    moved[source_name] = new_source
    moved[destination_name] = new_destination

    result = problem_text
    for name, point in moved.items():
        result = _set_latlon(result, name, point)

    for a, b in _read_edges(problem_text):
        if a not in moved or b not in moved:
            continue
        old_distance = _fact_value(problem_text, "distance", a, b)
        old_energy = _fact_value(problem_text, "energy-required", a, b)
        rate = (
            old_energy / old_distance
            if old_distance and old_energy is not None
            else DEFAULT_ENERGY_RATE
        )
        new_distance = haversine_m(moved[a], moved[b])
        if old_distance is not None:
            result = _set_fact_value(result, "distance", a, b, new_distance)
        if old_energy is not None:
            result = _set_fact_value(result, "energy-required", a, b, new_distance * rate)

    blocked = False
    via_points: list[LatLon] = []
    if no_fly_zone and len(no_fly_zone) >= 3:
        seg_a = _to_local(new_source, new_source)  # (0, 0), by construction
        seg_b = _to_local(new_destination, new_source)
        # The hull, not the raw click order: 4 corners clicked in a natural
        # (non-boundary-walk) order are a self-intersecting bowtie as a raw
        # polygon, which breaks every check below.
        polygon_local = _convex_hull([_to_local(p, new_source) for p in no_fly_zone])
        endpoint_inside = _point_in_polygon(seg_a, polygon_local) or _point_in_polygon(seg_b, polygon_local)
        if len(polygon_local) >= 3 and not endpoint_inside:
            # "Route around it" is meaningless if the mission starts or ends
            # inside the marked area - fly direct rather than force a detour
            # that can't actually avoid the zone it's departing/arriving in.
            blocked = _segment_crosses_polygon(seg_a, seg_b, polygon_local)
            if blocked:
                detour_local = _shortest_detour(seg_a, seg_b, polygon_local)
                via_points = [_from_local(p, new_source) for p in detour_local[1:-1]]

    if not blocked:
        result = _set_direct_route_only(result, source_name, destination_name)
    elif via_points:
        # A real shortest path around the polygon exists (the common case) -
        # use it instead of the template's fixed, unrelated detour shape.
        result = _set_detour_route(
            result, source_name, destination_name, new_source, new_destination, via_points
        )
    # else: blocked, but no via-points came back (a degenerate polygon) -
    # leave the template's own detour facts as the fallback.

    return result
