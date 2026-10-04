"""Configuration for the PSO path-planning assignment.

The seed is derived from the roll number (01-13622-060) so that every
student gets a different, but reproducible, problem instance.
"""

STUDENT_NAME = "Ayesha"
ROLL_NUMBER = "01-13622-060"
# Roll number with dashes removed -> integer seed: 0113622060 -> 113622060
SEED = int(ROLL_NUMBER.replace("-", ""))

# ---- Problem settings ----
GRID_SIZE = 20            # grid is GRID_SIZE x GRID_SIZE
OBSTACLE_DENSITY = 0.22   # fraction of cells that are blocked
MIN_START_GOAL_DIST = 15  # Manhattan distance between start and goal (keeps task non-trivial)

# ---- PSO settings ----
NUM_WAYPOINTS = 8         # intermediate waypoints per particle (dimension = 2 * NUM_WAYPOINTS)
SWARM_SIZE = 80
MAX_ITERATIONS = 400
W_START, W_END = 0.9, 0.4 # inertia weight, decreases linearly
C1 = 1.8                  # cognitive coefficient
C2 = 1.8                  # social coefficient
V_MAX_FRAC = 0.2          # max velocity as fraction of grid size
COLLISION_PENALTY = 50.0  # cost added per colliding sample point on the path
SAMPLE_STEP = 0.05        # step used when checking a segment for collisions
PATIENCE = 120            # stop early if no improvement for this many iterations
