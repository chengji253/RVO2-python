from KdTree import *
from Vector2 import *
from Agent import *
from Obstacle import *


class RVOSimulator:
    RVO_ERROR = float('inf')

    def __init__(self, timeStep=0.0, neighborDist=0.0, maxNeighbors=0, timeHorizon=0.0,
                 timeHorizonObst=0.0, radius=0.0, maxSpeed=0.0, velocity=None):
        self.agents_ = []
        self.obstacles_ = []

        self.obs_info = []
        self.goals_info = []

        self.defaultAgent_ = None
        self.kdTree_ = KdTree(self)

        self.globalTime_ = 0.0
        self.timeStep_ = timeStep

        if velocity is None:
            velocity = Vector2()

        if timeStep != 0.0:
            self.defaultAgent_ = Agent()
            self.defaultAgent_.maxNeighbors_ = maxNeighbors
            self.defaultAgent_.maxSpeed_ = maxSpeed
            self.defaultAgent_.neighborDist_ = neighborDist
            self.defaultAgent_.radius_ = radius
            self.defaultAgent_.timeHorizon_ = timeHorizon
            self.defaultAgent_.timeHorizonObst_ = timeHorizonObst
            self.defaultAgent_.velocity_ = velocity

    def __del__(self):
        if self.defaultAgent_:
            del self.defaultAgent_
        if self.kdTree_:
            del self.kdTree_
        for agent in self.agents_:
            del agent
        for obstacle in self.obstacles_:
            del obstacle

    def setTimeStep(self, timeStep):
        self.timeStep_ = timeStep

    def getGlobalTime(self):
        return self.globalTime_

    def getNumAgents(self):
        return len(self.agents_)

    def addAgent(self, position, neighborDist=None, maxNeighbors=None, timeHorizon=None,
                 timeHorizonObst=None, radius=None, maxSpeed=None, velocity=None):
        if self.defaultAgent_:
            if neighborDist is None:
                neighborDist = self.defaultAgent_.neighborDist_
            if maxNeighbors is None:
                maxNeighbors = self.defaultAgent_.maxNeighbors_
            if timeHorizon is None:
                timeHorizon = self.defaultAgent_.timeHorizon_
            if timeHorizonObst is None:
                timeHorizonObst = self.defaultAgent_.timeHorizonObst_
            if radius is None:
                radius = self.defaultAgent_.radius_
            if maxSpeed is None:
                maxSpeed = self.defaultAgent_.maxSpeed_
            if velocity is None:
                velocity = self.defaultAgent_.velocity_

            agent = Agent()
            agent.position_ = position
            agent.velocity_ = velocity
            agent.id_ = len(self.agents_)
            agent.maxNeighbors_ = maxNeighbors
            agent.maxSpeed_ = maxSpeed
            agent.neighborDist_ = neighborDist
            agent.radius_ = radius
            agent.timeHorizon_ = timeHorizon
            agent.timeHorizonObst_ = timeHorizonObst

            # self.agents_[agent.id_] = agent
            self.agents_.append(agent)

            return len(self.agents_) - 1
        return self.RVO_ERROR

    def addObstacle(self, vertices):
        if len(vertices) > 1:
            obstacleNo = len(self.obstacles_)

            # 遍历所有顶点
            for i in range(len(vertices)):
                obstacle = Obstacle()
                obstacle.point_ = vertices[i]

                if i != 0:
                    obstacle.previous_ = self.obstacles_[-1]
                    obstacle.previous_.next_ = obstacle
                if i == len(vertices) - 1:
                    obstacle.next_ = self.obstacles_[obstacleNo]
                    obstacle.next_.previous_ = obstacle

                next_index = 0 if i == len(vertices) - 1 else i + 1
                obstacle.direction_ = normalize(vertices[next_index] - vertices[i])

                if len(vertices) == 2:
                    obstacle.isConvex_ = True
                else:
                    prev_index = len(vertices) - 1 if i == 0 else i - 1
                    next_index = 0 if i == len(vertices) - 1 else i + 1
                    obstacle.isConvex_ = leftOf(vertices[prev_index], vertices[i], vertices[next_index]) >= 0
                obstacle.id_ = len(self.obstacles_)
                self.obstacles_.append(obstacle)
            return obstacleNo
        return self.RVO_ERROR

    def doStep(self):
        self.kdTree_.buildAgentTree()
        for agent in self.agents_:
            agent.computeNeighbors(self.kdTree_)
            agent.computeNewVelocity(self.timeStep_)
        self.agents_ = sorted(self.agents_, key=lambda x: x.id_)
        for agent in self.agents_:
            agent.update(self.timeStep_)
        self.globalTime_ += self.timeStep_

    def getAgentAgentNeighbor(self, agentNo, neighborNo):
        return self.agents_[agentNo].agentNeighbors_[neighborNo].second.id_

    def getAgentMaxNeighbors(self, agentNo):
        return self.agents_[agentNo].maxNeighbors_

    def getAgentMaxSpeed(self, agentNo):
        return self.agents_[agentNo].maxSpeed_

    def getAgentNeighborDist(self, agentNo):
        return self.agents_[agentNo].neighborDist_

    def getAgentNumAgentNeighbors(self, agentNo):
        return len(self.agents_[agentNo].agentNeighbors_)

    def getAgentNumObstacleNeighbors(self, agentNo):
        return len(self.agents_[agentNo].obstacleNeighbors_)

    def getAgentNumORCALines(self, agentNo):
        return len(self.agents_[agentNo].orcaLines_)

    def getAgentObstacleNeighbor(self, agentNo, neighborNo):
        return self.agents_[agentNo].obstacleNeighbors_[neighborNo].second.id_

    def getAgentORCALine(self, agentNo, lineNo):
        return self.agents_[agentNo].orcaLines_[lineNo]

    def getAgentPosition(self, agentNo):
        return self.agents_[agentNo].position_

    def getAgentPrefVelocity(self, agentNo):
        return self.agents_[agentNo].prefVelocity_

    def getAgentRadius(self, agentNo):
        return self.agents_[agentNo].radius_

    def getAgentTimeHorizon(self, agentNo):
        return self.agents_[agentNo].timeHorizon_

    def getAgentTimeHorizonObst(self, agentNo):
        return self.agents_[agentNo].timeHorizonObst_

    def getAgentVelocity(self, agentNo):
        return self.agents_[agentNo].velocity_

    def getObstacleVertex(self, vertexNo):
        return self.obstacles_[vertexNo].point_

    def getNextObstacleVertexNo(self, vertexNo):
        return self.obstacles_[vertexNo].next_.id_

    def getPrevObstacleVertexNo(self, vertexNo):
        return self.obstacles_[vertexNo].previous_.id_

    def processObstacles(self):
        self.kdTree_.buildObstacleTree()

    def queryVisibility(self, point1, point2, radius=0.0):
        return self.kdTree_.queryVisibility(point1, point2, radius)

    def setAgentDefaults(self, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed,
                         velocity=None):
        if velocity is None:
            velocity = Vector2()
        if self.defaultAgent_ is None:
            self.defaultAgent_ = Agent()
        self.defaultAgent_.maxNeighbors_ = maxNeighbors
        self.defaultAgent_.maxSpeed_ = maxSpeed
        self.defaultAgent_.neighborDist_ = neighborDist
        self.defaultAgent_.radius_ = radius
        self.defaultAgent_.timeHorizon_ = timeHorizon
        self.defaultAgent_.timeHorizonObst_ = timeHorizonObst
        self.defaultAgent_.velocity_ = velocity

    def setAgentMaxNeighbors(self, agentNo, maxNeighbors):
        self.agents_[agentNo].maxNeighbors_ = maxNeighbors

    def setAgentMaxSpeed(self, agentNo, maxSpeed):
        self.agents_[agentNo].maxSpeed_ = maxSpeed

    def setAgentNeighborDist(self, agentNo, neighborDist):
        self.agents_[agentNo].neighborDist_ = neighborDist

    def setAgentPosition(self, agentNo, position):
        self.agents_[agentNo].position_ = position

    def setAgentPrefVelocity(self, agentNo, prefVelocity):
        self.agents_[agentNo].prefVelocity_ = prefVelocity

    def setAgentRadius(self, agentNo, radius):
        self.agents_[agentNo].radius_ = radius

    def setAgentTimeHorizon(self, agentNo, timeHorizon):
        self.agents_[agentNo].timeHorizon_ = timeHorizon

    def setAgentTimeHorizonObst(self, agentNo, timeHorizonObst):
        self.agents_[agentNo].timeHorizonObst_ = timeHorizonObst

    def setAgentVelocity(self, agentNo, velocity):
        self.agents_[agentNo].velocity_ = velocity


    def obs_info_deal(self):
        obs_new = []
        for obs in self.obs_info:
            vertices = []
            for v in obs:
                o = (v.x_, v.y_)
                vertices.append(o)
            obs_new.append(vertices)

        self.obs_info = obs_new