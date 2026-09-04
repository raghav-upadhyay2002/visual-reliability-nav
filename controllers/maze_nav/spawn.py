"""Randomize the E-puck's start pose and the target Ball's position.

Walls in this project's maze convention always sit at cell *boundaries*,
never inside a cell, so any two distinct grid cells are always safe,
non-wall-embedded spawn points -- no wall-layout knowledge is needed here,
just the grid size and cell size. Those are read from the E-puck node's
`customData` field (set by tools/generate_maze_world.py as
"grid_size=N;cell_size=C"), so this works for any generator-style world
without per-world code changes.

Worlds outside that convention (e.g. testing_worlds/mazesolving_test.wbt,
hand-authored at a different scale) have no such customData; randomize_spawn
detects that and leaves the world's baked-in positions alone.
"""

import math
import random

from config import EPUCK_RESTING_Z, BALL_RESTING_Z, SPAWN_MIN_SEPARATION_FRACTION
from world_meta import read_world_meta


def _cell_x(c, cell):
    return -(c + 0.5) * cell


def _cell_y(n, r, cell):
    return -(n * cell) + cell * (r + 0.5)


def randomize_spawn(self_node, target_node, seed=None):
    """Teleport the E-puck and target Ball to random distinct grid cells.

    Returns (start_xy, target_xy) for logging, or None if this world doesn't
    carry the grid_size/cell_size customData this needs (positions are left
    untouched in that case).
    """
    meta = read_world_meta(self_node)
    if 'grid_size' not in meta or 'cell_size' not in meta:
        return None
    grid_size, cell = meta['grid_size'], meta['cell_size']

    rng = random.Random(seed)
    min_separation = grid_size * SPAWN_MIN_SEPARATION_FRACTION

    def random_cell():
        return rng.randrange(grid_size), rng.randrange(grid_size)

    start_cell = random_cell()
    target_cell = random_cell()
    while math.dist(start_cell, target_cell) < min_separation:
        target_cell = random_cell()

    start_xy = (_cell_x(start_cell[0], cell), _cell_y(grid_size, start_cell[1], cell))
    target_xy = (_cell_x(target_cell[0], cell), _cell_y(grid_size, target_cell[1], cell))
    start_rotation = rng.uniform(-math.pi, math.pi)

    self_node.getField('translation').setSFVec3f([start_xy[0], start_xy[1], EPUCK_RESTING_Z])
    self_node.getField('rotation').setSFRotation([0, 0, 1, start_rotation])
    self_node.resetPhysics()

    target_node.getField('translation').setSFVec3f([target_xy[0], target_xy[1], BALL_RESTING_Z])

    return start_xy, target_xy
