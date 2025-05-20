import random
import time
import os
from Vector2 import *
from RVOSimulator import *
from plotPic import *


RVO_OUTPUT_TIME_AND_POSITIONS = 1

RVO_TWO_PI = 2 * math.pi

def setup_scenario(simulator, goals):
    simulator.setTimeStep(0.25)

    simulator.setAgentDefaults(15.0, 10, 10.0, 10.0, 1.5, 2.0)

    for i in range(250):
        position = Vector2(200.0 * math.cos(i * RVO_TWO_PI * 0.004),
                               200.0 * math.sin(i * RVO_TWO_PI * 0.004))
        simulator.addAgent(position)
        goals.append(-simulator.getAgentPosition(i))


    simulator.goals_info = goals

def update_visualization(simulator):
    # print("time=" + str(simulator.getGlobalTime()))
    boundary = [[-210, 210], [-210, 210]]
    name = "test" + "/snap" + str(simulator.getGlobalTime()) + ".png"
    visualize_traj_dynamic(simulator.agents_, simulator.obs_info, simulator.goals_info, boundary, name)

def set_preferred_velocities(simulator, goals):
    for i in range(simulator.getNumAgents()):
        goal_vector = goals[i] - simulator.getAgentPosition(i)
        if absSq(goal_vector) > 1.0:
            goal_vector = normalize(goal_vector)
        simulator.setAgentPrefVelocity(i, goal_vector)

def reached_goal(simulator, goals):
    for i in range(simulator.getNumAgents()):
        if (simulator.getAgentPosition(i) - goals[i]) * (simulator.getAgentPosition(i) - goals[i]) > 100.0:
            return False
    return True

def mkdir_file(name_n):
    folder_path = os.path.dirname(name_n)
    if os.path.exists(folder_path):
        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.remove(file_path)
    else:
        os.makedirs(folder_path)

def main():
    goals = []
    simulator = RVOSimulator()

    setup_scenario(simulator, goals)

    name_n = "test" + "/snap" + str(2) + ".png"
    mkdir_file(name_n)
    t = 0
    while not reached_goal(simulator, simulator.goals_info):
        t += 1
        if t % 10 == 0:
            print("t=" + str(t))
            update_visualization(simulator)
        set_preferred_velocities(simulator, simulator.goals_info)
        simulator.doStep()

    del simulator


if __name__ == "__main__":
    main()
