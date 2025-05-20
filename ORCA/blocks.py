import math
import random
import time
import os
from Vector2 import *
from RVOSimulator import *
from plotPic import *


RVO_TWO_PI = 2 * math.pi


def setup_scenario(simulator, goals):
    random.seed(int(time.time()))

    simulator.setTimeStep(0.25)

    simulator.setAgentDefaults(15.0, 10, 5.0, 5.0, 2.0, 5.0)

    # simulator.addAgent(Vector2(55.0, 55.0))
    # goals.append(Vector2(-75.0, -75.0))
    for i in range(5):
        for j in range(5):
            simulator.addAgent(Vector2(55.0 + i * 10.0, 55.0 + j * 10.0))
            goals.append(Vector2(-75.0, -75.0))

            simulator.addAgent(Vector2(-55.0 - i * 10.0, 55.0 + j * 10.0))
            goals.append(Vector2(75.0, -75.0))

            simulator.addAgent(Vector2(55.0 + i * 10.0, -55.0 - j * 10.0))
            goals.append(Vector2(-75.0, 75.0))

            simulator.addAgent(Vector2(-55.0 - i * 10.0, -55.0 - j * 10.0))
            goals.append(Vector2(75.0, 75.0))

    obstacle1 = [Vector2(-10.0, 40.0), Vector2(-40.0, 40.0),
                 Vector2(-40.0, 10.0), Vector2(-10.0, 10.0)]
    obstacle2 = [Vector2(10.0, 40.0), Vector2(10.0, 10.0),
                 Vector2(40.0, 10.0), Vector2(40.0, 40.0)]
    obstacle3 = [Vector2(10.0, -40.0), Vector2(40.0, -40.0),
                 Vector2(40.0, -10.0), Vector2(10.0, -10.0)]
    obstacle4 = [Vector2(-10.0, -40.0), Vector2(-10.0, -10.0),
                 Vector2(-40.0, -10.0), Vector2(-40.0, -40.0)]

    simulator.obs_info.append(obstacle1)
    simulator.obs_info.append(obstacle2)
    simulator.obs_info.append(obstacle3)
    simulator.obs_info.append(obstacle4)
    simulator.obs_info_deal()
    simulator.goals_info = goals

    simulator.addObstacle(obstacle1)
    simulator.addObstacle(obstacle2)
    simulator.addObstacle(obstacle3)
    simulator.addObstacle(obstacle4)

    simulator.processObstacles()


def update_visualization(simulator):
    # print("time=" + str(simulator.getGlobalTime()))
    boundary = [[-100, 100], [-100, 100]]
    name = "test" + "/snap" + str(simulator.getGlobalTime()) + ".png"
    visualize_traj_dynamic(simulator.agents_, simulator.obs_info, simulator.goals_info, boundary, name)

    # for i in range(simulator.getNumAgents()):
    #     print(f" {simulator.getAgentPosition(i)}", end='')
    # print()

def mkdir_file(name_n):
    folder_path = os.path.dirname(name_n)
    if os.path.exists(folder_path):
        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.remove(file_path)
    else:
        os.makedirs(folder_path)


def set_preferred_velocities(simulator, goals):
    for i in range(simulator.getNumAgents()):
        goal_vector = goals[i] - simulator.getAgentPosition(i)
        if goal_vector * goal_vector > 1.0:
            goal_vector = goal_vector / abs(goal_vector)

        simulator.setAgentPrefVelocity(i, goal_vector)

        angle = random.random() * RVO_TWO_PI
        dist = random.random() * 0.0001
        perturb_vector = Vector2(math.cos(angle), math.sin(angle)) * dist
        simulator.setAgentPrefVelocity(i, simulator.getAgentPrefVelocity(i) + perturb_vector)


def reached_goal(simulator, goals):
    for i in range(simulator.getNumAgents()):
        if (simulator.getAgentPosition(i) - goals[i]) * (simulator.getAgentPosition(i) - goals[i]) > 100.0:
            return False
    return True


def main():
    goals = []

    simulator = RVOSimulator()

    setup_scenario(simulator, goals)

    name_n = "test" + "/snap" + str(2) + ".png"
    mkdir_file(name_n)
    t = 0
    while not reached_goal(simulator, goals):
        t += 1
        if t % 10 == 0:
            print("t=" + str(t))
            update_visualization(simulator)
        set_preferred_velocities(simulator, goals)
        simulator.doStep()

    del simulator


if __name__ == "__main__":
    main()