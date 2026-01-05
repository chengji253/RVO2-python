from Vector2 import *
from Obstacle import *
import copy

RVO_MAX_LEAF_SIZE = 10


class AgentTreeNode:
    def __init__(self):
        self.begin = 0
        self.end = 0
        self.left = 0
        self.right = 0
        self.maxX = 0.0
        self.maxY = 0.0
        self.minX = 0.0
        self.minY = 0.0


class ObstacleTreeNode:
    def __init__(self):
        self.obstacle_num = None
        self.left_obs_tree_node = None
        self.right_obs_tree_node = None

    def __del__(self):
        pass


class KdTree:

    def __init__(self, simulator):
        self.agents_ = []
        self.agentTree_ = []
        self.obstacleTree_ = None
        self.simulator_ = simulator

    def __del__(self):
        self.deleteObstacleTree(self.obstacleTree_)

    def buildAgentTree(self) -> None:
        if len(self.agents_) < len(self.simulator_.agents_):
            self.agents_.extend(self.simulator_.agents_[len(self.agents_):])
            self.agentTree_ = [AgentTreeNode() for _ in range(2 * len(self.agents_) - 1)]
        if self.agents_:
            self.buildAgentTreeRecursive(0, len(self.agents_), 0)

    def buildAgentTreeRecursive(self, begin, end, node):
        self.agentTree_[node].begin = begin
        self.agentTree_[node].end = end
        self.agentTree_[node].minX = self.agentTree_[node].maxX = self.agents_[begin].position_.x_
        self.agentTree_[node].minY = self.agentTree_[node].maxY = self.agents_[begin].position_.y_

        for i in range(begin + 1, end):
            self.agentTree_[node].maxX = max(self.agentTree_[node].maxX, self.agents_[i].position_.x_)
            self.agentTree_[node].minX = min(self.agentTree_[node].minX, self.agents_[i].position_.x_)
            self.agentTree_[node].maxY = max(self.agentTree_[node].maxY, self.agents_[i].position_.y_)
            self.agentTree_[node].minY = min(self.agentTree_[node].minY, self.agents_[i].position_.y_)

        if end - begin > RVO_MAX_LEAF_SIZE:
            isVertical = self.agentTree_[node].maxX - self.agentTree_[node].minX > \
                         self.agentTree_[node].maxY - self.agentTree_[node].minY
            splitValue = \
                0.5 * (self.agentTree_[node].maxX + self.agentTree_[node].minX) \
                    if isVertical else 0.5 * (self.agentTree_[node].maxY + self.agentTree_[node].minY)

            left = begin
            right = end

            while left < right:
                while left < right and \
                        ((isVertical and self.agents_[left].position_.x_ < splitValue) or (
                                not isVertical and self.agents_[left].position_.y_ < splitValue)):
                    left += 1
                while right > left and \
                        ((isVertical and self.agents_[right - 1].position_.x_ >= splitValue) or (
                                not isVertical and self.agents_[right - 1].position_.y_ >= splitValue)):
                    right -= 1
                if left < right:
                    self.agents_[left], self.agents_[right - 1] = self.agents_[right - 1], self.agents_[left]
                    left += 1
                    right -= 1
            if left == begin:
                left += 1
                right += 1

            self.agentTree_[node].left = node + 1
            self.agentTree_[node].right = node + 2 * (left - begin)

            self.buildAgentTreeRecursive(begin, left, self.agentTree_[node].left)
            self.buildAgentTreeRecursive(left, end, self.agentTree_[node].right)

    def buildObstacleTree(self):
        self.deleteObstacleTree(self.obstacleTree_)
        obstacles = self.simulator_.obstacles_
        self.obstacleTree_ = self.buildObstacleTreeRecursive(obstacles)

    def buildObstacleTreeRecursive(self, obstacles):
        if obstacles:
            node = ObstacleTreeNode()
            optimalSplit = 0
            minLeft = len(obstacles)
            minRight = len(obstacles)
            for i in range(len(obstacles)):
                leftSize = 0
                rightSize = 0
                obstacleI1 = obstacles[i]
                obstacleI2 = obstacleI1.next_
                for j in range(len(obstacles)):
                    if i != j:
                        obstacleJ1 = obstacles[j]
                        obstacleJ2 = obstacleJ1.next_
                        j1LeftOfI = leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_)
                        j2LeftOfI = leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_)
                        if j1LeftOfI >= -RVO_EPSILON and j2LeftOfI >= -RVO_EPSILON:
                            leftSize += 1
                        elif j1LeftOfI <= RVO_EPSILON and j2LeftOfI <= RVO_EPSILON:
                            rightSize += 1
                        else:
                            leftSize += 1
                            rightSize += 1
                        if (max(leftSize, rightSize), min(leftSize, rightSize)) >= (
                        max(minLeft, minRight), min(minLeft, minRight)):
                            break
                if (max(leftSize, rightSize), min(leftSize, rightSize)) < (
                max(minLeft, minRight), min(minLeft, minRight)):
                    minLeft = leftSize
                    minRight = rightSize
                    optimalSplit = i
            leftObstacles = [None] * minLeft
            rightObstacles = [None] * minRight
            leftCounter = 0
            rightCounter = 0
            i = optimalSplit
            obstacleI1 = obstacles[i]
            obstacleI2 = obstacleI1.next_
            for j in range(len(obstacles)):
                if i != j:
                    obstacleJ1 = obstacles[j]
                    obstacleJ2 = obstacleJ1.next_
                    j1LeftOfI = leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_)
                    j2LeftOfI = leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_)
                    if j1LeftOfI >= -RVO_EPSILON and j2LeftOfI >= -RVO_EPSILON:
                        leftObstacles[leftCounter] = obstacles[j]
                        leftCounter += 1
                    elif j1LeftOfI <= RVO_EPSILON and j2LeftOfI <= RVO_EPSILON:
                        rightObstacles[rightCounter] = obstacles[j]
                        rightCounter += 1
                    else:
                        t = det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleI1.point_) / det(
                            obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleJ2.point_)
                        splitPoint = obstacleJ1.point_ + t * (obstacleJ2.point_ - obstacleJ1.point_)
                        newObstacle = Obstacle()
                        newObstacle.direction_ = obstacleJ1.direction_
                        newObstacle.point_ = splitPoint
                        newObstacle.next_ = obstacleJ2
                        newObstacle.previous_ = obstacleJ1
                        newObstacle.id_ = len(self.simulator_.obstacles_)
                        newObstacle.isConvex_ = True
                        self.simulator_.obstacles_.append(newObstacle)
                        obstacleJ1.next_ = newObstacle
                        obstacleJ2.previous_ = newObstacle
                        if j1LeftOfI > 0.0:
                            leftObstacles[leftCounter] = obstacleJ1
                            leftCounter += 1
                            rightObstacles[rightCounter] = newObstacle
                            rightCounter += 1
                        else:
                            rightObstacles[rightCounter] = obstacleJ1
                            rightCounter += 1
                            leftObstacles[leftCounter] = newObstacle
                            leftCounter += 1
            node.obstacle = obstacleI1
            node.left = self.buildObstacleTreeRecursive(leftObstacles)
            node.right = self.buildObstacleTreeRecursive(rightObstacles)
            return node
        return None

    def computeAgentNeighbors(self, agent, rangeSq):
        rangeSq = self.queryAgentTreeRecursive(agent, rangeSq, 0)
        return rangeSq

    def computeObstacleNeighbors(self, agent, rangeSq):
        self.queryObstacleTreeRecursive(agent, rangeSq, self.obstacleTree_)

    def deleteObstacleTree(self, node):
        if node:
            self.deleteObstacleTree(node.left)
            self.deleteObstacleTree(node.right)
            del node

    def queryAgentTreeRecursive(self, agent, rangeSq, node):
        if self.agentTree_[node].end - self.agentTree_[node].begin <= RVO_MAX_LEAF_SIZE:
            for i in range(self.agentTree_[node].begin, self.agentTree_[node].end):

                rangeSq = agent.insertAgentNeighbor(self.agents_[i], rangeSq)
        else:
            distLeftMinX = max(0.0, self.agentTree_[self.agentTree_[node].left].minX - agent.position_.x_)
            distLeftMaxX = max(0.0, agent.position_.x_ - self.agentTree_[self.agentTree_[node].left].maxX)
            distLeftMinY = max(0.0, self.agentTree_[self.agentTree_[node].left].minY - agent.position_.y_)
            distLeftMaxY = max(0.0, agent.position_.y_ - self.agentTree_[self.agentTree_[node].left].maxY)
            distSqLeft = distLeftMinX * distLeftMinX + distLeftMaxX * distLeftMaxX + distLeftMinY * distLeftMinY + distLeftMaxY * distLeftMaxY
            distRightMinX = max(0.0, self.agentTree_[self.agentTree_[node].right].minX - agent.position_.x_)
            distRightMaxX = max(0.0, agent.position_.x_ - self.agentTree_[self.agentTree_[node].right].maxX)
            distRightMinY = max(0.0, self.agentTree_[self.agentTree_[node].right].minY - agent.position_.y_)
            distRightMaxY = max(0.0, agent.position_.y_ - self.agentTree_[self.agentTree_[node].right].maxY)
            distSqRight = distRightMinX * distRightMinX + distRightMaxX * distRightMaxX + distRightMinY * distRightMinY + distRightMaxY * distRightMaxY
            if distSqLeft < distSqRight:
                if distSqLeft < rangeSq:
                    rangeSq = self.queryAgentTreeRecursive(agent, rangeSq, self.agentTree_[node].left)
                    if distSqRight < rangeSq:
                        rangeSq = self.queryAgentTreeRecursive(agent, rangeSq, self.agentTree_[node].right)
            elif distSqRight < rangeSq:
                rangeSq = self.queryAgentTreeRecursive(agent, rangeSq, self.agentTree_[node].right)
                if distSqLeft < rangeSq:
                    rangeSq = self.queryAgentTreeRecursive(agent, rangeSq, self.agentTree_[node].left)
        return rangeSq

    def queryObstacleTreeRecursive(self, agent, rangeSq, node):
        if node:
            obstacle1 = node.obstacle
            obstacle2 = obstacle1.next_
            agentLeftOfLine = leftOf(obstacle1.point_, obstacle2.point_, agent.position_)
            self.queryObstacleTreeRecursive(agent, rangeSq, node.left if agentLeftOfLine >= 0.0 else node.right)
            distSqLine = agentLeftOfLine * agentLeftOfLine / absSq(obstacle2.point_ - obstacle1.point_)
            if distSqLine < rangeSq:
                if agentLeftOfLine < 0.0:
                    agent.insertObstacleNeighbor(node.obstacle, rangeSq)
                self.queryObstacleTreeRecursive(agent, rangeSq, node.right if agentLeftOfLine >= 0.0 else node.left)

    def queryVisibility(self, vector1, vector2, radius):
        return self.queryVisibilityRecursive(vector1, vector2, radius, self.obstacleTree_)

    def queryVisibilityRecursive(self, vector1, vector2, radius, node):
        if node:
            obstacle1 = node.obstacle
            obstacle2 = obstacle1.next_
            q1LeftOfI = leftOf(obstacle1.point_, obstacle2.point_, vector1)
            q2LeftOfI = leftOf(obstacle1.point_, obstacle2.point_, vector2)
            invLengthI = 1.0 / absSq(obstacle2.point_ - obstacle1.point_)
            if q1LeftOfI >= 0.0 and q2LeftOfI >= 0.0:
                return self.queryVisibilityRecursive(vector1, vector2, radius, node.left) and ((
               q1LeftOfI * q1LeftOfI * invLengthI >= radius * radius and q2LeftOfI * q2LeftOfI * invLengthI >= radius * radius) or self.queryVisibilityRecursive(
                    vector1, vector2, radius, node.right))
            if q1LeftOfI <= 0.0 and q2LeftOfI <= 0.0:
                return self.queryVisibilityRecursive(vector1, vector2, radius, node.right) and ((
                q1LeftOfI * q1LeftOfI * invLengthI >= radius * radius and q2LeftOfI * q2LeftOfI * invLengthI >= radius * radius) or self.queryVisibilityRecursive(
                    vector1, vector2, radius, node.left))
            if q1LeftOfI >= 0.0 and q2LeftOfI <= 0.0:
                return self.queryVisibilityRecursive(vector1, vector2, radius,
                 node.left) and self.queryVisibilityRecursive(vector1, vector2, radius, node.right)
            point1LeftOfQ = leftOf(vector1, vector2, obstacle1.point_)
            point2LeftOfQ = leftOf(vector1, vector2, obstacle2.point_)
            invLengthQ = 1.0 / absSq(vector2 - vector1)
            return point1LeftOfQ * point2LeftOfQ >= 0.0 and \
               point1LeftOfQ * point1LeftOfQ * invLengthQ > radius * radius and point2LeftOfQ * point2LeftOfQ * invLengthQ > radius * radius and self.queryVisibilityRecursive(
                vector1, vector2, radius, node.left) and self.queryVisibilityRecursive(vector1, vector2, radius,
                                                                                       node.right)
        return True
