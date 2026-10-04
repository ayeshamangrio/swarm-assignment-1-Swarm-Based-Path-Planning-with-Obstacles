# Draw this on paper, photograph it, save as docs/flow_diagram.jpg

Boxes top to bottom (use arrows):

1. START
2. Set seed = 113622060 (from roll no. 01-13622-060)
3. Generate 20x20 grid -> place 88 random obstacles -> pick start & goal on free cells (BFS check)
4. Initialise swarm: 80 particles = 8 waypoints each (random noise around the start-goal line), random velocities
5. Build path: start -> waypoints -> goal
6. Evaluate fitness = path length + 50 x collisions (check each segment against obstacle cells)
7. Update pbest (each particle) and gbest (swarm)
8. Update velocity: v = w*v + c1*r1*(pbest-x) + c2*r2*(gbest-x); update position x = x + v (clip to grid)
9. DECISION: max iterations (400) reached OR no improvement for 120 iterations?
      NO  -> go back to step 5
      YES -> continue
10. Output best path, length, collisions
11. Plot grid, obstacles, S, G, path (matplotlib)
12. END
