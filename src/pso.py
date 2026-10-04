"""Particle Swarm Optimisation for 2D path planning.

Particle encoding
-----------------
Each particle is a vector of K intermediate waypoints (x1,y1,...,xK,yK) in
continuous grid coordinates. The full path is

    start -> wp1 -> wp2 -> ... -> wpK -> goal   (straight segments)

Fitness (to minimise)
---------------------
    cost = total path length + COLLISION_PENALTY * (#sample points inside obstacles)

A path with zero collisions is feasible; its cost is exactly its length.
"""
import numpy as np


class PathPSO:
    def __init__(self, blocked, start, goal, n, cfg, seed):
        self.n = n
        self.cfg = cfg
        self.rng = np.random.RandomState(seed)
        # cell centres in continuous coordinates
        self.start = np.array(start, dtype=float) + 0.5
        self.goal = np.array(goal, dtype=float) + 0.5
        self.mask = np.zeros((n, n), dtype=bool)
        for (x, y) in blocked:
            self.mask[x, y] = True
        self.dim = 2 * cfg.NUM_WAYPOINTS
        self.vmax = cfg.V_MAX_FRAC * n

    # ---------- path helpers ----------
    def decode(self, particle):
        wps = particle.reshape(-1, 2)
        return np.vstack([self.start, wps, self.goal])

    def collisions(self, path):
        """Count sampled points on the polyline that lie in blocked cells."""
        count = 0
        for a, b in zip(path[:-1], path[1:]):
            seg_len = np.linalg.norm(b - a)
            steps = max(2, int(seg_len / self.cfg.SAMPLE_STEP))
            t = np.linspace(0, 1, steps)[:, None]
            pts = a + t * (b - a)
            idx = np.clip(np.floor(pts).astype(int), 0, self.n - 1)
            count += int(self.mask[idx[:, 0], idx[:, 1]].sum())
        return count

    @staticmethod
    def length(path):
        return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))

    def fitness(self, particle):
        path = self.decode(particle)
        return self.length(path) + self.cfg.COLLISION_PENALTY * self.collisions(path)

    # ---------- main loop ----------
    def run(self, verbose=True):
        c = self.cfg
        K = c.NUM_WAYPOINTS
        # Initialise: waypoints spread along the start-goal line + large random noise
        base = np.linspace(self.start, self.goal, K + 2)[1:-1].ravel()
        pos = base + self.rng.normal(0, self.n * 0.25, (c.SWARM_SIZE, self.dim))
        pos = np.clip(pos, 0, self.n - 1e-3)
        vel = self.rng.uniform(-self.vmax, self.vmax, (c.SWARM_SIZE, self.dim))

        pbest = pos.copy()
        pbest_f = np.array([self.fitness(p) for p in pos])
        g = int(np.argmin(pbest_f))
        gbest, gbest_f = pbest[g].copy(), pbest_f[g]

        history = [gbest_f]
        stall = 0
        for it in range(c.MAX_ITERATIONS):
            w = c.W_START - (c.W_START - c.W_END) * it / c.MAX_ITERATIONS
            r1 = self.rng.rand(c.SWARM_SIZE, self.dim)
            r2 = self.rng.rand(c.SWARM_SIZE, self.dim)
            # velocity update: inertia + cognitive + social
            vel = w * vel + c.C1 * r1 * (pbest - pos) + c.C2 * r2 * (gbest - pos)
            vel = np.clip(vel, -self.vmax, self.vmax)
            # position update + keep inside the grid
            pos = np.clip(pos + vel, 0, self.n - 1e-3)

            f = np.array([self.fitness(p) for p in pos])
            better = f < pbest_f
            pbest[better], pbest_f[better] = pos[better], f[better]

            g = int(np.argmin(pbest_f))
            if pbest_f[g] < gbest_f - 1e-9:
                gbest, gbest_f, stall = pbest[g].copy(), pbest_f[g], 0
            else:
                stall += 1
            history.append(gbest_f)

            if verbose and (it % 25 == 0 or it == c.MAX_ITERATIONS - 1):
                print(f"iter {it:4d} | best cost = {gbest_f:8.3f}")
            if stall >= c.PATIENCE:  # stopping condition
                if verbose:
                    print(f"Early stop at iteration {it}: no improvement for {c.PATIENCE} iterations")
                break

        best_path = self.decode(gbest)
        return best_path, gbest_f, history
