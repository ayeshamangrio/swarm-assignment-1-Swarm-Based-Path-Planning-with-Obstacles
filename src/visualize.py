"""Matplotlib visualisation of the grid, obstacles, start, goal and best path."""
import matplotlib.pyplot as plt


def plot_solution(blocked, start, goal, path, n, title, save_path, show=False):
    fig, ax = plt.subplots(figsize=(7, 7))
    for (x, y) in blocked:
        ax.add_patch(plt.Rectangle((x, y), 1, 1, color="black"))
    ax.add_patch(plt.Rectangle(start, 1, 1, color="green", alpha=0.8))
    ax.add_patch(plt.Rectangle(goal, 1, 1, color="red", alpha=0.8))
    ax.plot(path[:, 0], path[:, 1], "-o", color="dodgerblue", lw=2.2, ms=4, label="PSO path")
    ax.text(start[0] + .5, start[1] + .5, "S", ha="center", va="center", color="white", weight="bold")
    ax.text(goal[0] + .5, goal[1] + .5, "G", ha="center", va="center", color="white", weight="bold")
    ax.set_xlim(0, n); ax.set_ylim(0, n); ax.set_aspect("equal")
    ax.set_xticks(range(n + 1)); ax.set_yticks(range(n + 1))
    ax.grid(True, color="lightgray", lw=0.5)
    ax.tick_params(labelbottom=False, labelleft=False)
    ax.set_title(title)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def plot_convergence(history, save_path):
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(history, color="darkorange", lw=2)
    ax.set_xlabel("Iteration"); ax.set_ylabel("Global best cost")
    ax.set_title("PSO convergence"); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
