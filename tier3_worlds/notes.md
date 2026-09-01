# Tier 3 (structurally-independent held-out) worlds

Mazes built by a different process than `training_worlds/`'s generator, used to
test whether the reliability/navigation method generalizes beyond the wall-
placement bias of our own generator. Keep everything here separate from
`training_worlds/` and `testing_worlds/` — never train on these.

De-prioritized relative to the corruption module + evaluation pipeline (the
actual bottleneck for the 3-week update) — pick this back up once that's done.

## Status

- **`mazesolving_test.wbt`** — done. Adapted from the "Webots Maze-Solving
  Robot Simulation" community repo (see Source 2 below). Controller swapped to
  `maze_nav`, cameras added, target `Ball` placed at the same spot as the
  maze's own red target wall. **Not yet opened in Webots to verify it loads
  cleanly / is fully navigable** — do that before layering corruption on top.
- **Maze-Robot-on-Webots** — dropped. Its `Maze.wbt` is tagged
  `#VRML_SIM V8.5 utf8` (~2015-era Webots, predates the `R20XXx` versioning
  scheme). Webots R2025a refuses to open it at all ("Invalid header") and
  can't auto-upgrade it — properly migrating a file this old means stepping
  through several intermediate Webots versions by hand, which combined with
  its Y-up axis convention (see below) isn't worth the effort for one held-
  out maze. Not pursued further.
- **Rat's Life** — on hold. `contest_manager.exe` stalled at t=0 in a local
  test run (never advanced past `0:00:00:000`), likely stuck in
  `generate_random_maze()`'s retry loop before ever calling its first
  simulation step. Not debugged further — see Source 1 below if revisiting.
- **Webots-Wall_Following_Robot** — dropped. Turns out to not be a maze at
  all: it's a 5m x 5.5m outdoor city/pedestrian scene (10 pedestrian NPCs,
  "sanitizer" and "junction_emitter" controllers) with a custom 4-wheeled
  robot, not an e-puck. No usable maze structure to extract.

## Naming

- `ratslife_test_NN.wbt` — frozen snapshots of Webots' built-in Rat's Life
  contest maze (see below).
- `<source>_test.wbt` — one file per adopted community maze repo, `<source>`
  = short slug of the repo name.

## Source 1: Rat's Life (`File > Open Sample Worlds > contests > ratslife`)

Rat's Life is **not** a set of 10 static maze files — it's one world
(`ratslife.wbt`) whose `contest_manager` supervisor procedurally rearranges
120 movable wall/interval bricks into a **random** maze via
`srand(time(NULL))` on every launch (see `maze_generator.c`). To get a
reproducible held-out layout from it:

1. In Webots: open the sample world, press play, let it run ~1-2s until the
   bricks stop moving (maze fully built), then pause.
2. `File > Save World As` into this folder as `ratslife_test_01.wbt` — this
   bakes the current brick positions in as literal coordinates and removes
   the random-generation code path. Repeat with fresh launches for `_02`,
   `_03`, etc. if more than one layout is wanted.
3. Adapt the saved file so it matches our robot setup:
   - Delete the second E-puck (`Rat1`), its battery/feeder nodes, and the
     `contest_manager` supervisor `Robot` node — only the maze geometry is
     needed.
   - On the remaining E-puck: `controller "Rat0"` → `controller "maze_nav"`,
     remove `battery [...]`, replace the `E-puckFlag` turretSlot entry with
     the `camera_left`/`camera_right` Camera nodes used in
     `training_worlds/train_maze_01.wbt` (lines ~852-869).
   - Add a `Ball` node as the target (Rat's Life has none).
   - Reskin `LegoWall`/`LegoInterval` appearance to match our `Roughcast`
     `colorOverride` walls, so wall **texture** stays constant and only maze
     **topology** varies — otherwise appearance and structure differences
     are confounded.

Known issue: in a local test, `contest_manager.exe` never advanced past sim
time 0 — likely stuck in `generate_random_maze()`'s retry loop. Rat0/Rat1
also fail to start (missing `java`/`javaw` on PATH) but that's unrelated and
harmless, since maze-building is driven entirely by `contest_manager`, not
the two robot controllers.

## Source 2: community repos

Three candidates were evaluated by actually opening their world files (not
just descriptions):

- **Maze-Robot-on-Webots** (`haoransh/Maze-Robot-on-Webots`, `Maze.wbt`) —
  **dropped**. Genuine hand-built e-puck maze, ~3m x 3m, and used the old
  Y-up axis convention (pre-dates Webots' switch to Z-up around R2022b —
  its Viewpoint/E-puck translations have the near-zero "height" coordinate
  in Y, not Z, unlike everything else in this project). Turned out to also
  be tagged `#VRML_SIM V8.5 utf8` (~2015-era Webots), which R2025a refuses
  to open ("Invalid header") and can't auto-upgrade — migrating it means
  stepping through several intermediate Webots versions by hand. Combined
  with the axis-convention risk, not worth it for one held-out maze. If
  ever revisited: the file also has **no floor/ground plane at all**, so
  after a successful version migration one would still need to add a floor
  sized to the wall layout's extent, plus the same controller/camera/target
  adaptation as `mazesolving_test.wbt`.
- **Webots Maze-Solving Robot Simulation**
  (`JeewanthaSadaruwan/Webots---Maze-Solving-Robot-Simulation`) — done, see
  `mazesolving_test.wbt` above. 2.5m x 2.5m arena, e-puck, confirmed
  different wall-spacing convention from our generator. Its local
  `protos/E-puck.proto` turned out to be an unmodified vendored copy of the
  standard Cyberbotics R2023b E-puck proto, so the adapted file references
  the standard GitHub-hosted proto instead of carrying a local copy.
- **Webots-Wall_Following_Robot** (`pasindu-ud/Webots-Wall_Following_Robot`)
  — dropped, not a maze (see Status above).

Scale caveat: neither adopted maze is at the same scale as our ~1m x 1m
generator output (repo 1 ~3m x 3m, repo 2 2.5m x 2.5m). `maze_nav.py`'s
edge-density and brightness thresholds
([controllers/maze_nav/maze_nav.py:122](../controllers/maze_nav/maze_nav.py#L122),
[:184](../controllers/maze_nav/maze_nav.py#L184)) were tuned for our maze's
wall-to-camera distance regime and may not read walls correctly at these
larger scales — this conflates "structural difference" with "scale
mismatch." Worth checking once loaded in Webots; may need threshold retuning
or geometry rescaling to properly isolate topology as the only variable.
