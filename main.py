"""Entry point: generate the roll-number-seeded problem, run PSO, plot results.

Usage:  python main.py            (saves images to results/)
        python main.py --show     (also opens the matplotlib window)
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "src"))

import config as cfg
from grid import generate_problem
from pso import PathPSO
from visualize import plot_solution, plot_convergence


def main():
    show = "--show" in sys.argv
    print(f"Student : {cfg.STUDENT_NAME}")
    print(f"Roll No : {cfg.ROLL_NUMBER}")
    print(f"Seed    : {cfg.SEED}")

    blocked, start, goal = generate_problem(cfg.SEED, cfg.GRID_SIZE,
                                            cfg.OBSTACLE_DENSITY, cfg.MIN_START_GOAL_DIST)
    print(f"Grid {cfg.GRID_SIZE}x{cfg.GRID_SIZE} | obstacles: {len(blocked)} | start: {start} | goal: {goal}\n")

    pso = PathPSO(blocked, start, goal, cfg.GRID_SIZE, cfg, seed=cfg.SEED)
    path, cost, history = pso.run()

    n_coll = pso.collisions(path)
    length = pso.length(path)
    straight = ((goal[0] - start[0]) ** 2 + (goal[1] - start[1]) ** 2) ** 0.5
    print("\n===== RESULT =====")
    print(f"Fitness cost     : {cost:.3f}")
    print(f"Path length      : {length:.3f}  (straight-line lower bound: {straight:.3f})")
    print(f"Collision samples: {n_coll}  ->  {'FEASIBLE (obstacle-free)' if n_coll == 0 else 'INFEASIBLE'}")
    print("Waypoints:")
    for i, (x, y) in enumerate(path):
        print(f"  {i}: ({x:.2f}, {y:.2f})")

    os.makedirs("results", exist_ok=True)
    plot_solution(blocked, start, goal, path, cfg.GRID_SIZE,
                  f"PSO path | seed={cfg.SEED} | length={length:.2f}", "results/path.png", show=show)
    plot_convergence(history, "results/convergence.png")
    print("\nSaved: results/path.png, results/convergence.png")


if __name__ == "__main__":
    main()
