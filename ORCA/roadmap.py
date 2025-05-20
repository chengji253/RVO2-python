import math
import random
import time
import os
from Vector2 import *
from RVOSimulator import *
from plotPic import *
import heapq
import json


# 控制是否输出时间和位置
RVO_OUTPUT_TIME_AND_POSITIONS = 1
# 控制是否种子随机数生成器
RVO_SEED_RANDOM_NUMBER_GENERATOR = 1

# 定义常量
RVO_TWO_PI = 2 * math.pi

class RoadmapVertex:
    def __init__(self):
        self.position = None
        self.neighbors = []
        self.distToGoal = []

def setup_scenario(simulator, roadmap, goals):
    if RVO_SEED_RANDOM_NUMBER_GENERATOR:
        random.seed()

    # 设定全局时间步长
    simulator.setTimeStep(0.25)

    # 添加多边形障碍物
    obstacle1 = [Vector2(-10.0, 40.0), Vector2(-40.0, 40.0),
                 Vector2(-40.0, 10.0), Vector2(-10.0, 10.0)]
    obstacle2 = [Vector2(10.0, 40.0), Vector2(10.0, 10.0),
                 Vector2(40.0, 10.0), Vector2(40.0, 40.0)]
    obstacle3 = [Vector2(10.0, -40.0), Vector2(40.0, -40.0),
                 Vector2(40.0, -10.0), Vector2(10.0, -10.0)]
    obstacle4 = [Vector2(-10.0, -40.0), Vector2(-10.0, -10.0),
                 Vector2(-40.0, -10.0), Vector2(-40.0, -40.0)]

    simulator.addObstacle(obstacle1)
    simulator.addObstacle(obstacle2)
    simulator.addObstacle(obstacle3)
    simulator.addObstacle(obstacle4)

    # 处理障碍物
    simulator.processObstacles()



    # 添加路线图顶点
    goal_positions = [Vector2(-75.0, -75.0), Vector2(75.0, -75.0),
                      Vector2(-75.0, 75.0), Vector2(75.0, 75.0)]
    for pos in goal_positions:
        v = RoadmapVertex()
        v.position = pos
        roadmap.append(v)

    vertex_positions = [
        Vector2(-42.0, -42.0), Vector2(-42.0, -8.0), Vector2(-42.0, 8.0), Vector2(-42.0, 42.0),
        Vector2(-8.0, -42.0), Vector2(-8.0, -8.0), Vector2(-8.0, 8.0), Vector2(-8.0, 42.0),
        Vector2(8.0, -42.0), Vector2(8.0, -8.0), Vector2(8.0, 8.0), Vector2(8.0, 42.0),
        Vector2(42.0, -42.0), Vector2(42.0, -8.0), Vector2(42.0, 8.0), Vector2(42.0, 42.0)
    ]
    for pos in vertex_positions:
        v = RoadmapVertex()
        v.position = pos
        roadmap.append(v)

    # 设定代理的默认参数
    simulator.setAgentDefaults(15.0, 10, 5.0, 5.0, 2.0, 2.0)

    # 添加代理并存储目标
    for i in range(5):
        for j in range(5):
            simulator.addAgent(Vector2(55.0 + i * 10.0, 55.0 + j * 10.0))
            goals.append(0)

            simulator.addAgent(Vector2(-55.0 - i * 10.0, 55.0 + j * 10.0))
            goals.append(1)

            simulator.addAgent(Vector2(55.0 + i * 10.0, -55.0 - j * 10.0))
            goals.append(2)

            simulator.addAgent(Vector2(-55.0 - i * 10.0, -55.0 - j * 10.0))
            goals.append(3)

    simulator.obs_info.append(obstacle1)
    simulator.obs_info.append(obstacle2)
    simulator.obs_info.append(obstacle3)
    simulator.obs_info.append(obstacle4)
    simulator.obs_info_deal()
    simulator.goals_info = goals

    return roadmap, goals


def update_visualization(simulator):
    # 输出当前全局时间
    # print("time=" + str(simulator.getGlobalTime()))
    boundary = [[-100, 100], [-100, 100]]
    name = "test" + "/snap" + str(simulator.getGlobalTime()) + ".png"
    visualize_traj_dynamic(simulator.agents_, simulator.obs_info, simulator.goals_info, boundary ,name)

def build_roadmap(simulator, roadmap):
    # 连接路线图顶点
    for i in range(len(roadmap)):
        for j in range(len(roadmap)):
            if simulator.queryVisibility(roadmap[i].position, roadmap[j].position, simulator.getAgentRadius(0)):
                roadmap[i].neighbors.append(j)

        roadmap[i].distToGoal = [float('inf')] * 4

    for i in range(4):
        Q = []
        visited = [False] * len(roadmap)

        roadmap[i].distToGoal[i] = 0.0
        heapq.heappush(Q, (0.0, i))

        while Q:
            _, u = heapq.heappop(Q)
            if visited[u]:
                continue
            visited[u] = True

            for j in roadmap[u].neighbors:
                distUV = abs(roadmap[j].position - roadmap[u].position)

                if roadmap[j].distToGoal[i] > roadmap[u].distToGoal[i] + distUV:
                    roadmap[j].distToGoal[i] = roadmap[u].distToGoal[i] + distUV
                    heapq.heappush(Q, (roadmap[j].distToGoal[i], j))

def set_preferred_velocities(simulator, roadmap, goals):
    for i in range(simulator.getNumAgents()):
        minDist = float('inf')
        minVertex = -1

        for j in range(len(roadmap)):
            dist = abs(roadmap[j].position - simulator.getAgentPosition(i)) + roadmap[j].distToGoal[goals[i]]
            if dist < minDist and simulator.queryVisibility(simulator.getAgentPosition(i), roadmap[j].position, simulator.getAgentRadius(i)):
                minDist = dist
                minVertex = j

        if minVertex == -1:
            simulator.setAgentPrefVelocity(i, Vector2(0.0, 0.0))
        else:
            if absSq(roadmap[minVertex].position - simulator.getAgentPosition(i)) == 0.0:
                if minVertex == goals[i]:
                    simulator.setAgentPrefVelocity(i, Vector2())
                else:
                    simulator.setAgentPrefVelocity(i, normalize(roadmap[goals[i]].position - simulator.getAgentPosition(i)))
            else:
                simulator.setAgentPrefVelocity(i, normalize(roadmap[minVertex].position - simulator.getAgentPosition(i)))


        angle = random.random() * RVO_TWO_PI
        dist = random.random() * 0.0001
        pref_vel = simulator.getAgentPrefVelocity(i)
        perturb = Vector2(math.cos(angle), math.sin(angle)) * dist
        simulator.setAgentPrefVelocity(i, pref_vel + perturb)
        # simulator.setAgentPrefVelocity(i, pref_vel)

def reached_goal(simulator, roadmap, goals):
    # 检查所有代理是否到达目标
    for i in range(simulator.getNumAgents()):
        if absSq(simulator.getAgentPosition(i) - roadmap[goals[i]].position) > 400.0:
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
    roadmap = []
    goals = []

    data_all = []
    simulator = RVOSimulator()

    roadmap, goals = setup_scenario(simulator, roadmap, goals)

    build_roadmap(simulator, roadmap)
    name_n = "test" + "/snap" + str(2) + ".png"
    mkdir_file(name_n)
    # 执行并操作模拟
    t = 0
    while not reached_goal(simulator, roadmap, goals):
        t += 1
        if t % 10 == 0:
            print("t=" + str(t))
            if RVO_OUTPUT_TIME_AND_POSITIONS:
                update_visualization(simulator)
        set_preferred_velocities(simulator, roadmap, goals)
        simulator.doStep()

    del simulator
    return 0

if __name__ == "__main__":
    main()