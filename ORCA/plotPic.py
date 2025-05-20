import matplotlib
import matplotlib.pyplot as pyplot
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import matplotlib.cm as cmx
import matplotlib.colors as colors
import numpy as np
import matplotlib.pyplot as plt


def get_cmap(N):
    '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
    color_norm = colors.Normalize(vmin=0, vmax=N - 1)
    scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv')

    def map_index_to_rgb_color(index):
        return scalar_map.to_rgba(index)

    return map_index_to_rgb_color


def visualize_traj_dynamic(agents, obs_info, goals, boundary, name):
    matplotlib.use('Agg')
    matplotlib.rcParams['path.simplify'] = True
    matplotlib.rcParams['path.simplify_threshold'] = 1.0
    matplotlib.rcParams['figure.dpi'] = 300

    figure = plt.figure(figsize=(7, 10))
    ax = figure.add_subplot(1, 1, 1)
    ax.grid(True, linestyle='--', alpha=0.7)
    cmap = get_cmap(len(agents))

    for vertices in obs_info:
        poly = patches.Polygon(
            vertices,
            facecolor='palegreen',
            edgecolor='darkgreen',
            linewidth=2,
            fill=True,
            alpha=1,
            zorder=2
        )
        ax.add_patch(poly)

    legend_elements = []
    for i in range(0, len(agents)):
        robot = patches.Circle(
            (agents[i].position_.x_, agents[i].position_.y_),
            radius=agents[i].radius_,
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=0.8,
            ls='solid',
            alpha=0.9,
            zorder=2)

        ax.add_patch(robot)
        # ax.text(agents[i].position_.x_, agents[i].position_.y_, str(i),
        #         fontsize=12, color='black', ha='center', va='center')
        # ax.arrow(agents[i].position_.x_, agents[i].position_.y_,
        #          agents[i].velocity_.x_*5, agents[i].velocity_.y_*5, head_width=1.5, head_length=2, fc=cmap(i), ec=cmap(i), alpha=0.8)
        if hasattr(goals[i], 'x_') and hasattr(goals[i], 'y_'):
            goal_marker, = ax.plot([goals[i].x_], [goals[i].y_], '*', color=cmap(i), markersize=8, linewidth=3.0)
        # path = swarm.des_path_pos[i]
        # path = np.array(path)
        # path_line, = plt.plot(path[:, 0], path[:, 1], color=cmap(i), linewidth=1.2)

    # if time:
    #     ax.text(0, boundary[1][1] + 1, f'$t={time:.1f} s$', fontsize=20, fontweight='bold')
    # out_size = 2
    ax.set_aspect('equal')
    # boundary = [[-100, 100], [-100, 100]]
    out_size = 0
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size - 1)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size + 3)

    if name:
        plt.savefig(name, dpi=300, bbox_inches='tight')
    plt.cla()
    plt.close(figure)
    return figure