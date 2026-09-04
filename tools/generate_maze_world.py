"""Generate (or convert) maze layouts for this project's Webots worlds.

Reverse-engineered from the existing training_worlds/*.svg + *.wbt pairs, so
output matches their exact grid convention, wall-node naming, and coordinate
math. Two modes:

  generate  Carve a fresh NxN perfect maze (randomized DFS) from a seed, and
            emit both the .svg (visualization, same dialect as the existing
            files) and the .wbt (Webots world) for it.
  convert   Take an existing hand-authored .svg in the same dialect and emit
            just the matching .wbt, without generating new wall layout.

Grid/coordinate convention (derived from training_worlds/train_maze_01.wbt
and cross-checked against train_maze_02/05/09, all of which use a 0.1m cell):
  - An NxN grid of `cell`-meter cells (--cell-size, default 0.15 -- wider than
    the original 0.1m convention, chosen to give the 0.075m-diameter e-puck
    more physical clearance in a corridor; see [[feedback-maze-complexity]]).
    SVG uses 10-unit cells regardless of physical cell size (grid line at
    x=k*10 <-> Webots x offset k*cell); col/row are 0-indexed, 0 at the
    SVG-coordinate-origin corner.
  - Only *internal* walls (between adjacent cells) are ever emitted as Wall
    nodes; the maze's outer perimeter is provided by RectangleArena's own
    default boundary walls, not by anything this script writes.
  - "e" wall = vertical wall between cell (c, r) and (c+1, r):
      translation (-(c+1)*cell, -(N*cell) + cell*(r + 0.5), 0),
      size "0.01 cell 0.1" (thickness/height fixed regardless of cell size;
      only the cell-spanning length dimension scales)
  - "s" wall = horizontal wall between cell (c, r) and (c, r+1):
      translation (-(c + 0.5)*cell, -(N*cell) + cell*(r + 1), 0),
      size "cell 0.01 0.1"
  - Cell center (used for the E-puck start and Ball target):
      x = -(c + 0.5) * cell
      y = -(N*cell) + cell*(r + 0.5)   (same formula as the "e" wall's y)
  - RectangleArena: translation (-N*cell/2, -N*cell/2, 0), floorSize
    (N*cell, N*cell) -- sized to exactly match the maze extent (the original
    files used a fixed "1 1" floor, which only worked because every sampled
    maze was <=10 cells at 0.1m; this generalizes it for arbitrary cell size).
  - Viewpoint: position (arena.x - 2*cell, arena.y, 2*cell*N), orientation
    fixed -- extrapolated from the fixed-cell samples to keep the same
    framing proportions as cell size changes.
  - E-puck resting z = 0.0248092, Ball resting z = 0.030379 -- constant across
    every sampled maze (proto resting heights, independent of cell size).

Usage:
  python tools/generate_maze_world.py generate --size 8 --seed 42 \
      --out testing_worlds/test_maze_13

  python tools/generate_maze_world.py convert --svg some_maze.svg \
      --out testing_worlds/test_maze_14 --cell-size 0.15
"""
import argparse
import random
import re
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

DEFAULT_CELL = 0.15  # meters per grid cell -- wider than the original 0.1m
                      # convention, for more e-puck clearance (see module docstring)
WALL_THICKNESS = 0.01
WALL_HEIGHT = 0.1


# --------------------------------------------------------------------------
# Maze representation: walls is a set of ("e", c, r) / ("s", c, r) tuples for
# every *closed* internal boundary. Passages are simply the complement over
# all valid internal boundaries for an NxN grid.
# --------------------------------------------------------------------------

def all_internal_walls(n):
    e = {("e", c, r) for c in range(n - 1) for r in range(n)}
    s = {("s", c, r) for c in range(n) for r in range(n - 1)}
    return e | s


def carve_maze(n, seed):
    """Randomized depth-first carve -> a perfect (fully-connected, loop-free)
    maze. Returns the set of closed walls (all internal boundaries minus the
    carved passages)."""
    rng = random.Random(seed)
    visited = [[False] * n for _ in range(n)]

    def unvisited_neighbors(c, r):
        options = []
        if c + 1 < n and not visited[r][c + 1]:
            options.append((("e", c, r), (c + 1, r)))
        if c - 1 >= 0 and not visited[r][c - 1]:
            options.append((("e", c - 1, r), (c - 1, r)))
        if r + 1 < n and not visited[r + 1][c]:
            options.append((("s", c, r), (c, r + 1)))
        if r - 1 >= 0 and not visited[r - 1][c]:
            options.append((("s", c, r - 1), (c, r - 1)))
        return options

    start = (rng.randrange(n), rng.randrange(n))
    visited[start[1]][start[0]] = True
    stack = [start]
    open_passages = set()

    while stack:
        c, r = stack[-1]
        options = unvisited_neighbors(c, r)
        if not options:
            stack.pop()
            continue
        wall_key, nxt = rng.choice(options)
        open_passages.add(wall_key)
        visited[nxt[1]][nxt[0]] = True
        stack.append(nxt)

    return all_internal_walls(n) - open_passages


# --------------------------------------------------------------------------
# SVG <-> wall-set
# --------------------------------------------------------------------------

def svg_dialect(n, walls):
    dim = n * 10 + 20
    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<svg xmlns="http://www.w3.org/2000/svg"',
        '    xmlns:xlink="http://www.w3.org/1999/xlink"',
        f'    width="{dim}" height="{dim}" viewBox="-10 -10 {dim} {dim}">',
        '<defs>',
        '<style type="text/css"><![CDATA[',
        'line {',
        '    stroke-width: 2;',
        '}',
        ']]></style>',
        '</defs>',
    ]
    for kind, c, r in sorted(walls):
        if kind == "e":
            x = (c + 1) * 10
            lines.append(f'<line x1="{x}" y1="{r * 10}" x2="{x}" y2="{(r + 1) * 10}" stroke="green"/>')
        else:
            y = (r + 1) * 10
            lines.append(f'<line x1="{c * 10}" y1="{y}" x2="{(c + 1) * 10}" y2="{y}" stroke="red"/>')
    lines.append(f'<line x1="0" y1="0" x2="{n * 10}" y2="0"/>')
    lines.append(f'<line x1="0" y1="0" x2="0" y2="{n * 10}"/>')
    lines.append('</svg>')
    return "\n".join(lines) + "\n"


def parse_svg(path):
    """Parse an existing maze SVG (same dialect) back into (n, walls)."""
    tree = ET.parse(path)
    root = tree.getroot()
    ns = {"svg": "http://www.w3.org/2000/svg"}
    walls = set()
    max_coord = 0
    for line in root.findall(".//svg:line", ns) or root.findall(".//line"):
        stroke = line.get("stroke")
        if not stroke:
            continue  # unstyled border-decoration lines, not real walls
        x1, y1, x2, y2 = (float(line.get(a)) for a in ("x1", "y1", "x2", "y2"))
        max_coord = max(max_coord, x1, y1, x2, y2)
        if stroke == "green":  # vertical -> "e" wall
            if x1 != x2:
                raise ValueError(f"green line not vertical: {line.attrib}")
            c = round(x1 / 10) - 1
            r = round(min(y1, y2) / 10)
            walls.add(("e", c, r))
        elif stroke == "red":  # horizontal -> "s" wall
            if y1 != y2:
                raise ValueError(f"red line not horizontal: {line.attrib}")
            r = round(y1 / 10) - 1
            c = round(min(x1, x2) / 10)
            walls.add(("s", c, r))
    n = round(max_coord / 10)
    return n, walls


# --------------------------------------------------------------------------
# Grid geometry helpers (shared by WBT rendering + start/target placement)
# --------------------------------------------------------------------------

def cell_x(c, cell):
    return -(c + 0.5) * cell


def cell_y(n, r, cell):
    return -(n * cell) + cell * (r + 0.5)


def e_wall_translation(n, c, r, cell):
    return (-(c + 1) * cell, cell_y(n, r, cell), 0)


def s_wall_translation(n, c, r, cell):
    return (cell_x(c, cell), -(n * cell) + cell * (r + 1), 0)


def farthest_pair(n, walls):
    """BFS-twice over the passage graph to get a long, guaranteed-reachable
    start/target pair (falls back gracefully if the maze isn't fully
    connected -- e.g. a hand-edited SVG passed to `convert`)."""
    def neighbors(c, r):
        candidates = (
            (("e", c, r), (c + 1, r)),
            (("e", c - 1, r), (c - 1, r)),
            (("s", c, r), (c, r + 1)),
            (("s", c, r - 1), (c, r - 1)),
        )
        for wall_key, (nc, nr) in candidates:
            if 0 <= nc < n and 0 <= nr < n and wall_key not in walls:
                yield (nc, nr)

    def bfs(start):
        dist = {start: 0}
        q = deque([start])
        while q:
            cur = q.popleft()
            for nb in neighbors(*cur):
                if nb not in dist:
                    dist[nb] = dist[cur] + 1
                    q.append(nb)
        farthest = max(dist, key=dist.get)
        return farthest, dist[farthest]

    a, _ = bfs((0, 0))
    b, _ = bfs(a)
    return a, b


# --------------------------------------------------------------------------
# WBT rendering
# --------------------------------------------------------------------------

WBT_HEADER = """#VRML_SIM R2025a utf8
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/apartment_structure/protos/Wall.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/balls/protos/Ball.proto"
WorldInfo {{
info [
    "Simple e-puck simulation that can be controlled with ROS2."
]
    title "ROS2 simulation of the e-puck robot"
}}
Viewpoint {{
    orientation 0 2 0 1.52
    position {vx} {vy} {vz}
    follow "e-puck"
}}
TexturedBackground {{
}}

TexturedBackgroundLight {{
  castShadows FALSE
}}

Solid {{
    children [

        RectangleArena {{
            translation {ax} {ay} 0
            rotation 1 0 0 0
            floorSize {floor_size} {floor_size}
            wallAppearance PBRAppearance {{
                baseColor {wr} {wg} {wb}
                roughness 1
                metalness 0
            }}
            floorAppearance PBRAppearance {{
                baseColor {fr} {fg} {fb}
                roughness 1
                metalness 0
            }}
        }}
"""

WALL_TMPL = """
		Wall {{
			translation  {x} {y}  0
			rotation 1 0 0 0
			name "wall_{c}_{r}_{kind}"
			size {sx} {sy} {sz}
			appearance PBRAppearance {{
				baseColor {wr} {wg} {wb}
				roughness 1
				metalness 0
			}}
		}}
"""

FOOTER_TMPL = """	]
}}
E-puck {{
  translation {ex} {ey} 0.0248092
  rotation 0 0 1 {erot}
  controller "{controller}"
  supervisor TRUE
  camera_width 256
  camera_height 192
  turretSlot [
    Camera {{
      translation 0 0.02 0
      rotation 0 0 1 1.5708
      name "camera_left"
      fieldOfView 0.84
      width 256
      height 192
    }}
    Camera {{
      translation 0 -0.02 0
      rotation 0 0 1 -1.5708
      name "camera_right"
      fieldOfView 0.84
      width 256
      height 192
    }}
  ]
}}
DEF TARGET_BALL Ball {{
  translation {tx} {ty} 0.030379
  color 1 0 0
}}
"""


def render_wbt(n, walls, start, target, controller="maze_nav", start_rotation=4.71239,
                wall_color=(0.15, 0.35, 0.75), floor_color=(0.95, 0.85, 0.88),
                cell=DEFAULT_CELL):
    ax = ay = round(-n * cell / 2, 10)
    floor_size = round(n * cell, 10)
    vx, vy, vz = round(ax - 2 * cell, 10), ay, round(2 * cell * n, 10)
    wr, wg, wb = wall_color
    fr, fg, fb = floor_color

    out = [WBT_HEADER.format(vx=vx, vy=vy, vz=vz, ax=ax, ay=ay, floor_size=floor_size,
                              wr=wr, wg=wg, wb=wb, fr=fr, fg=fg, fb=fb)]

    # Positions on the outer rim (c == n-1 for "e", r == n-1 for "s") sit
    # exactly on RectangleArena's own boundary wall -- the reference worlds
    # never emit a separate Wall node there, so skip them here too (a
    # `convert`-parsed SVG may still contain per-cell lines drawn along that
    # rim purely for visualization; those are dropped, not the interior).
    interior_walls = [(kind, c, r) for kind, c, r in walls
                       if not (kind == "e" and c == n - 1) and not (kind == "s" and r == n - 1)]

    for kind, c, r in sorted(interior_walls):
        if kind == "e":
            x, y, _ = e_wall_translation(n, c, r, cell)
            sx, sy, sz = WALL_THICKNESS, cell, WALL_HEIGHT
        else:
            x, y, _ = s_wall_translation(n, c, r, cell)
            sx, sy, sz = cell, WALL_THICKNESS, WALL_HEIGHT
        out.append(WALL_TMPL.format(x=x, y=y, c=c, r=r, kind=kind, sx=sx, sy=sy, sz=sz,
                                     wr=wr, wg=wg, wb=wb))

    ex, ey = cell_x(start[0], cell), cell_y(n, start[1], cell)
    tx, ty = cell_x(target[0], cell), cell_y(n, target[1], cell)
    out.append(FOOTER_TMPL.format(ex=ex, ey=ey, erot=start_rotation, controller=controller,
                                   tx=tx, ty=ty))
    return "".join(out)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def parse_cell_arg(value):
    c, r = value.split(",")
    return (int(c), int(r))


def parse_color_arg(value):
    r, g, b = value.split(",")
    return (float(r), float(g), float(b))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)

    gen = sub.add_parser("generate", help="carve a fresh maze and emit .svg + .wbt")
    gen.add_argument("--size", type=int, default=8, help="grid size NxN (default: 8, per project convention)")
    gen.add_argument("--seed", type=int, required=True, help="RNG seed, for reproducibility")

    conv = sub.add_parser("convert", help="convert an existing maze .svg to .wbt only")
    conv.add_argument("--svg", required=True, help="path to an existing maze .svg")

    for p in (gen, conv):
        p.add_argument("--out", required=True, help="output path without extension, e.g. testing_worlds/test_maze_13")
        p.add_argument("--controller", default="maze_nav")
        p.add_argument("--start", type=parse_cell_arg, default=None, help="'col,row' -- default: auto-picked far pair")
        p.add_argument("--target", type=parse_cell_arg, default=None, help="'col,row' -- default: auto-picked far pair")
        p.add_argument("--start-rotation", type=float, default=4.71239)
        p.add_argument("--wall-color", type=parse_color_arg, default=None)
        p.add_argument("--floor-color", type=parse_color_arg, default=None)
        p.add_argument("--cell-size", type=float, default=DEFAULT_CELL,
                        help=f"meters per grid cell (default: {DEFAULT_CELL})")

    args = parser.parse_args()

    if args.mode == "generate":
        n = args.size
        walls = carve_maze(n, args.seed)
        svg_text = svg_dialect(n, walls)
    else:
        n, walls = parse_svg(args.svg)
        svg_text = None  # not regenerated -- input file is already the source of truth

    if args.start and args.target:
        start, target = args.start, args.target
    else:
        start, target = farthest_pair(n, walls)

    wall_color = args.wall_color or (0.15, 0.35, 0.75)
    floor_color = args.floor_color or (0.95, 0.85, 0.88)

    wbt_text = render_wbt(n, walls, start, target, controller=args.controller,
                           start_rotation=args.start_rotation,
                           wall_color=wall_color, floor_color=floor_color,
                           cell=args.cell_size)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if svg_text is not None:
        out.with_suffix(".svg").write_text(svg_text, encoding="utf-8")
        print(f"wrote {out.with_suffix('.svg')}")
    out.with_suffix(".wbt").write_text(wbt_text, encoding="utf-8")
    print(f"wrote {out.with_suffix('.wbt')}  (grid {n}x{n}, start {start}, target {target})")


if __name__ == "__main__":
    main()
