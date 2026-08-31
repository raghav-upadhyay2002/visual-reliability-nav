# Tier 3 (structurally-independent held-out) worlds

Mazes built by a different process than `training_worlds/`'s generator, used to
test whether the reliability/navigation method generalizes beyond the wall-
placement bias of our own generator. Keep everything here separate from
`training_worlds/` and `testing_worlds/` — never train on these.

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

## Source 2: community repo (decide after eyeballing Rat's Life variants)

Candidates, pick 0 or 1 only if Rat's Life alone doesn't give enough
structural diversity (different corridor width / turn density / dead-end
frequency from our generator):

- **Maze-Robot-on-Webots** (`Maze.wbt`) — hand-built e-puck maze
- **Webots Maze-Solving Robot Simulation** — 2.5m x 2.5m arena, different
  wall-spacing convention
- **Webots-Wall_Following_Robot** (`maze.wbt`) — another independently-built
  maze

To add one: `git clone` it into a scratch location, copy just the `.wbt`
plus any `.proto`/texture assets it references into this folder, then apply
the same controller/camera/target adaptation steps as above.
