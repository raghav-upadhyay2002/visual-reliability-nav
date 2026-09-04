"""Read per-world metadata (grid size, cell size, wall/floor color) that
tools/generate_maze_world.py bakes into the E-puck's `customData` field as
"key=value;key=value;...".

Colors are floats in [0,1] as multi-value "r,g,b" -- Webots' own baseColor
convention -- so detectors can key off each maze's *actual* rendered colors
instead of one hardcoded constant that only happens to match one maze.

Worlds outside this convention (e.g. testing_worlds/mazesolving_test.wbt,
hand-authored at a different scale) simply have no customData; callers get
back only whichever keys were present, if any, and should have a fallback.
"""


def read_world_meta(self_node):
    custom_data = self_node.getField('customData')
    raw = custom_data.getSFString() if custom_data else ''

    meta = {}
    for kv in raw.split(';'):
        if '=' not in kv:
            continue
        key, value = kv.split('=', 1)
        if ',' in value:
            meta[key] = tuple(float(x) for x in value.split(','))
        else:
            try:
                meta[key] = int(value)
            except ValueError:
                try:
                    meta[key] = float(value)
                except ValueError:
                    meta[key] = value
    return meta
