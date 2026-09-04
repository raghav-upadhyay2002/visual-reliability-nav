"""Per-trial CSV logging.

Log location is configurable via env vars so batch runs (e.g. sweeping
corruption severities) can redirect output without editing this file.
"""

import csv
import datetime
import os

LOG_HEADER = [
    'sim_time_s',
    'wall_ahead', 'wall_left', 'wall_front_right', 'wall_right',
    'left_density', 'center_density', 'right_density', 'mean_right',
    'target_visible', 'target_direction',
    'left_velocity', 'right_velocity',
    # eval-only, from the supervisor/proximity sensors -- never fed back into
    # the navigation decisions above
    'dist_to_target', 'collided', 'outcome',
]


class TrialLogger:
    def __init__(self):
        log_dir = os.environ.get('MAZE_NAV_LOG_DIR', os.path.dirname(os.path.abspath(__file__)))
        log_name = os.environ.get(
            'MAZE_NAV_LOG_NAME',
            'run_log_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv',
        )
        self.path = os.path.join(log_dir, log_name)

        self._file = open(self.path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(LOG_HEADER)

    def log_row(self, row):
        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        self._file.close()
