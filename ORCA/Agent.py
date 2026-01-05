import math
from Vector2 import *
from Line import *
import copy


class Agent:
    def __init__(self):
        self.agentNeighbors_ = []
        self.obstacleNeighbors_ = []
        self.orcaLines_ = []

        self.newVelocity_ = Vector2()
        self.position_ = Vector2()
        self.prefVelocity_ = Vector2()
        self.velocity_ = Vector2()

        self.id_ = 0
        self.maxNeighbors_ = 0

        self.maxSpeed_ = 0.0
        self.neighborDist_ = 0.0
        self.radius_ = 0.0
        self.timeHorizon_ = 0.0
        self.timeHorizonObst_ = 0.0

    def computeNeighbors(self, kdTree):
        self.obstacleNeighbors_.clear()
        range = self.timeHorizonObst_ * self.maxSpeed_ + self.radius_
        kdTree.computeObstacleNeighbors(self, range * range)
        self.agentNeighbors_.clear()

        if self.maxNeighbors_ > 0:
            rangeSq = self.neighborDist_ * self.neighborDist_
            kdTree.computeAgentNeighbors(self, rangeSq)

    def computeNewVelocity(self, timeStep):
        # / *Search for the best new velocity.* /
        self.orcaLines_.clear()
        invTimeHorizonObst = 1.0 / self.timeHorizonObst_

        for i in range(len(self.obstacleNeighbors_)):
            obstacle1 = self.obstacleNeighbors_[i][1]
            obstacle2 = obstacle1.next_
            relativePosition1 = obstacle1.point_ - self.position_
            relativePosition2 = obstacle2.point_ - self.position_

            alreadyCovered = False
            for j in range(len(self.orcaLines_)):
                if (
                        det(relativePosition1 * invTimeHorizonObst - self.orcaLines_[j].point,
                            self.orcaLines_[j].direction) - invTimeHorizonObst * self.radius_ >= -RVO_EPSILON
                        and det(invTimeHorizonObst * relativePosition2 - self.orcaLines_[j].point,
                                self.orcaLines_[j].direction) - invTimeHorizonObst * self.radius_ >= -RVO_EPSILON
                ):
                    alreadyCovered = True
                    break
            if alreadyCovered:
                continue

            distSq1 = absSq(relativePosition1)
            distSq2 = absSq(relativePosition2)
            radiusSq = self.radius_ * self.radius_
            obstacleVector = obstacle2.point_ - obstacle1.point_
            s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector)
            distSqLine = absSq(-relativePosition1 - obstacleVector * s)

            line = Line()
            if s < 0.0 and distSq1 <= radiusSq:
                if obstacle1.isConvex_:
                    line.point = Vector2(0.0, 0.0)
                    line.direction = normalize(Vector2(-relativePosition1.y_, relativePosition1.x_))
                    self.orcaLines_.append(line)
                continue

            if s > 1.0 and distSq2 <= radiusSq:
                if obstacle2.isConvex_ and det(relativePosition2, obstacle2.direction_) >= 0.0:
                    line.point = Vector2(0.0, 0.0)
                    line.direction = normalize(Vector2(-relativePosition2.y_, relativePosition2.x_))
                    self.orcaLines_.append(line)
                continue

            if s >= 0.0 and s <= 1.0 and distSqLine <= radiusSq:
                line.point = Vector2(0.0, 0.0)
                line.direction = -obstacle1.direction_
                self.orcaLines_.append(line)
                continue

            # leftLegDirection = Vector2
            # rightLegDirection = Vector2

            if s < 0.0 and distSqLine <= radiusSq:
                if not obstacle1.isConvex_:
                    continue
                obstacle2 = obstacle1
                leg1 = math.sqrt(distSq1 - radiusSq)
                leftLegDirection = Vector2(
                    relativePosition1.x_ * leg1 - relativePosition1.y_ * self.radius_,
                    relativePosition1.x_ * self.radius_ + relativePosition1.y_ * leg1
                ) / distSq1
                rightLegDirection = Vector2(
                    relativePosition1.x_ * leg1 + relativePosition1.y_ * self.radius_,
                    -relativePosition1.x_ * self.radius_ + relativePosition1.y_ * leg1
                ) / distSq1
            elif s > 1.0 and distSqLine <= radiusSq:
                if not obstacle2.isConvex_:
                    continue
                obstacle1 = obstacle2
                leg2 = math.sqrt(distSq2 - radiusSq)
                leftLegDirection = Vector2(
                    relativePosition2.x_ * leg2 - relativePosition2.y_ * self.radius_,
                    relativePosition2.x_ * self.radius_ + relativePosition2.y_ * leg2
                ) / distSq2
                rightLegDirection = Vector2(
                    relativePosition2.x_ * leg2 + relativePosition2.y_ * self.radius_,
                    -relativePosition2.x_ * self.radius_ + relativePosition2.y_ * leg2
                ) / distSq2
            else:
                if obstacle1.isConvex_:
                    leg1 = math.sqrt(distSq1 - radiusSq)
                    leftLegDirection = Vector2(
                        relativePosition1.x_ * leg1 - relativePosition1.y_ * self.radius_,
                        relativePosition1.x_ * self.radius_ + relativePosition1.y_ * leg1
                    ) / distSq1
                else:
                    leftLegDirection = -obstacle1.direction_
                if obstacle2.isConvex_:
                    leg2 = math.sqrt(distSq2 - radiusSq)
                    rightLegDirection = Vector2(
                        relativePosition2.x_ * leg2 + relativePosition2.y_ * self.radius_,
                        -relativePosition2.x_ * self.radius_ + relativePosition2.y_ * leg2
                    ) / distSq2
                else:
                    rightLegDirection = obstacle1.direction_

            leftNeighbor = obstacle1.previous_
            isLeftLegForeign = False
            isRightLegForeign = False
            if obstacle1.isConvex_ and det(leftLegDirection, -leftNeighbor.direction_) >= 0.0:
                leftLegDirection = -leftNeighbor.direction_
                isLeftLegForeign = True
            if obstacle2.isConvex_ and det(rightLegDirection, obstacle2.direction_) <= 0.0:
                rightLegDirection = obstacle2.direction_
                isRightLegForeign = True

            leftCutoff = (obstacle1.point_ - self.position_) * invTimeHorizonObst
            rightCutoff = (obstacle2.point_ - self.position_) * invTimeHorizonObst
            cutoffVector = rightCutoff - leftCutoff

            t = (
                0.5
                if obstacle1 == obstacle2
                else (self.velocity_ - leftCutoff) * cutoffVector / absSq(cutoffVector)
            )
            tLeft = (self.velocity_ - leftCutoff) * leftLegDirection
            tRight = (self.velocity_ - rightCutoff) * rightLegDirection

            if (t < 0.0 and tLeft < 0.0) or (obstacle1 == obstacle2 and tLeft < 0.0 and tRight < 0.0):
                unitW = normalize(self.velocity_ - leftCutoff)
                line.direction = Vector2(unitW.y_, -unitW.x_)
                line.point = leftCutoff + self.radius_ * invTimeHorizonObst * unitW
                self.orcaLines_.append(line)
                continue
            if t > 1.0 and tRight < 0.0:
                unitW = normalize(self.velocity_ - rightCutoff)
                line.direction = Vector2(unitW.y_, -unitW.x_)
                line.point = rightCutoff + unitW * self.radius_ * invTimeHorizonObst
                self.orcaLines_.append(line)
                continue

            distSqCutoff = (
                float('inf')
                if (t < 0.0 or t > 1.0 or obstacle1 == obstacle2)
                else absSq(self.velocity_ - (leftCutoff + t * cutoffVector))
            )
            distSqLeft = (
                float('inf')
                if tLeft < 0.0
                else absSq(self.velocity_ - (leftCutoff + tLeft * leftLegDirection))
            )
            distSqRight = (
                float('inf')
                if tRight < 0.0
                else absSq(self.velocity_ - (rightCutoff + tRight * rightLegDirection))
            )

            if distSqCutoff <= distSqLeft and distSqCutoff <= distSqRight:
                line.direction = -obstacle1.direction_
                line.point = leftCutoff + Vector2(-line.direction.y_,
                                                  line.direction.x_) * self.radius_ * invTimeHorizonObst
                self.orcaLines_.append(line)
                continue
            if distSqLeft <= distSqRight:
                if isLeftLegForeign:
                    continue
                line.direction = leftLegDirection
                line.point = leftCutoff + Vector2(-line.direction.y_,
                                                  line.direction.x_) * self.radius_ * invTimeHorizonObst
                self.orcaLines_.append(line)
                continue
            if isRightLegForeign:
                continue
            line.direction = -rightLegDirection
            line.point = rightCutoff + Vector2(-line.direction.y_,
                                               line.direction.x_) * self.radius_ * invTimeHorizonObst
            self.orcaLines_.append(line)

        numObstLines = len(self.orcaLines_)
        invTimeHorizon = 1.0 / self.timeHorizon_

        # 创建代理ORCA线
        for i in range(len(self.agentNeighbors_)):
            other = self.agentNeighbors_[i][1]
            relativePosition = other.position_ - self.position_
            relativeVelocity = self.velocity_ - other.velocity_
            distSq = absSq(relativePosition)
            combinedRadius = self.radius_ + other.radius_
            combinedRadiusSq = combinedRadius * combinedRadius

            line = Line()
            if distSq > combinedRadiusSq:
                w = relativeVelocity - relativePosition * invTimeHorizon
                wLengthSq = absSq(w)
                dotProduct = w * relativePosition
                if dotProduct < 0.0 and dotProduct * dotProduct > combinedRadiusSq * wLengthSq:
                    wLength = math.sqrt(wLengthSq)
                    unitW = w / wLength
                    line.direction = Vector2(unitW.y_, -unitW.x_)
                    u = unitW * (combinedRadius * invTimeHorizon - wLength)
                else:
                    leg = math.sqrt(distSq - combinedRadiusSq)
                    if det(relativePosition, w) > 0.0:
                        line.direction = Vector2(
                            relativePosition.x_ * leg - relativePosition.y_ * combinedRadius,
                            relativePosition.x_ * combinedRadius + relativePosition.y_ * leg
                        ) / distSq
                    else:
                        line.direction = -Vector2(
                            relativePosition.x_ * leg + relativePosition.y_ * combinedRadius,
                            -relativePosition.x_ * combinedRadius + relativePosition.y_ * leg
                        ) / distSq
                    u = line.direction * (relativeVelocity * line.direction) - relativeVelocity
            else:
                invTimeStep = 1.0 / timeStep
                w = relativeVelocity - invTimeStep * relativePosition
                wLength = abs(w)
                unitW = w / wLength
                line.direction = Vector2(unitW.y_, -unitW.x_)
                u = (combinedRadius * invTimeStep - wLength) * unitW

            line.point = self.velocity_ + u * 0.5
            self.orcaLines_.append(line)

        lineFail, self.newVelocity_ = \
            linearProgram2(self.orcaLines_, self.maxSpeed_, self.prefVelocity_, False, self.newVelocity_)

        if lineFail < len(self.orcaLines_):
            self.newVelocity_ = linearProgram3(self.orcaLines_, numObstLines, lineFail, self.maxSpeed_, self.newVelocity_)

    def insertAgentNeighbor(self, agent: 'Agent', rangeSq: float) -> float:
        if self != agent:
            distSq = absSq(self.position_ - agent.position_)
            if distSq < rangeSq:
                if len(self.agentNeighbors_) < self.maxNeighbors_:
                    self.agentNeighbors_.append((distSq, agent))
                i = len(self.agentNeighbors_) - 1
                while i != 0 and distSq < self.agentNeighbors_[i - 1][0]:
                    self.agentNeighbors_[i] = self.agentNeighbors_[i - 1]
                    i -= 1
                self.agentNeighbors_[i] = (distSq, agent)
                if len(self.agentNeighbors_) == self.maxNeighbors_:
                    rangeSq = self.agentNeighbors_[-1][0]
        return rangeSq

    def insertObstacleNeighbor(self, obstacle, rangeSq):
        nextObstacle = obstacle.next_
        r = ((self.position_ - obstacle.point_) * (nextObstacle.point_ - obstacle.point_)) / absSq(
            nextObstacle.point_ - obstacle.point_)
        if r < 0.0:
            distSq = absSq(self.position_ - obstacle.point_)
        elif r > 1.0:
            distSq = absSq(self.position_ - nextObstacle.point_)
        else:
            distSq = absSq(self.position_ - (obstacle.point_ + r * (nextObstacle.point_ - obstacle.point_)))
        if distSq < rangeSq:
            self.obstacleNeighbors_.append((distSq, obstacle))
            i = len(self.obstacleNeighbors_) - 1
            while i != 0 and distSq < self.obstacleNeighbors_[i - 1][0]:
                self.obstacleNeighbors_[i] = self.obstacleNeighbors_[i - 1]
                i -= 1
            self.obstacleNeighbors_[i] = (distSq, obstacle)

    def update(self, timeStep):
        self.velocity_ = self.newVelocity_
        self.position_ += self.velocity_ * timeStep


def linearProgram1(lines, lineNo, radius, optVelocity, directionOpt, result):
    dotProduct = lines[lineNo].point * lines[lineNo].direction
    discriminant = dotProduct * dotProduct + radius * radius - absSq(lines[lineNo].point)

    if discriminant < 0.0:
        # Max speed circle fully invalidates line lineNo.
        return False, result

    sqrtDiscriminant = math.sqrt(discriminant)
    tLeft = -dotProduct - sqrtDiscriminant
    tRight = -dotProduct + sqrtDiscriminant

    for i in range(lineNo):
        denominator = det(lines[lineNo].direction, lines[i].direction)
        numerator = det(lines[i].direction, lines[lineNo].point - lines[i].point)

        if abs(denominator) <= RVO_EPSILON:
            # Lines lineNo and i are (almost) parallel.
            if numerator < 0.0:
                return False, result
            continue

        t = numerator / denominator

        if denominator >= 0.0:
            # Line i bounds line lineNo on the right.
            tRight = min(tRight, t)
        else:
            # Line i bounds line lineNo on the left.
            tLeft = max(tLeft, t)

        if tLeft > tRight:
            return False, result

    if directionOpt:
        # Optimize direction.
        if optVelocity * lines[lineNo].direction > 0.0:
            # Take right extreme.
            result = lines[lineNo].point + lines[lineNo].direction * tRight
        else:
            # Take left extreme.
            result = lines[lineNo].point + lines[lineNo].direction * tLeft
    else:
        # Optimize closest point.
        t = lines[lineNo].direction * (optVelocity - lines[lineNo].point)

        if t < tLeft:
            result = lines[lineNo].point + lines[lineNo].direction * tLeft
        elif t > tRight:
            result = lines[lineNo].point + lines[lineNo].direction * tRight
        else:
            result = lines[lineNo].point + lines[lineNo].direction * t

    return True, result


def linearProgram2(lines, radius, optVelocity, directionOpt, result):
    if directionOpt:
        # Optimize direction. Note that the optimization velocity is of unit length
        # in this case.
        result = optVelocity * radius
    elif absSq(optVelocity) > radius * radius:
        # Optimize closest point and outside circle.
        result = normalize(optVelocity) * radius
    else:
        # Optimize closest point and inside circle.
        result = optVelocity

    for i in range(len(lines)):
        if det(lines[i].direction, lines[i].point - result) > 0.0:
            # Result does not satisfy constraint i. Compute new optimal result.
            tempResult = copy.copy(result)
            success, new_result = linearProgram1(lines, i, radius, optVelocity, directionOpt, result)
            if not success:
                result = copy.copy(tempResult)
                return i, result
            result = new_result

    return len(lines), result


"""
 * @relates        Agent
 * @brief          Solves a two-dimensional linear program subject to linear
 *                 constraints defined by lines and a circular constraint.
 * @param[in]      lines        Lines defining the linear constraints.
 * @param[in]      numObstLines Count of obstacle lines.
 * @param[in]      beginLine    The line on which the 2-d linear program failed.
 * @param[in]      radius       The radius of the circular constraint.
 * @param[in, out] result       A reference to the result of the linear program.
"""


# linearProgram3 函数
def linearProgram3(lines, numObstLines, beginLine, radius, result):
    distance = 0.0

    # if result is None:
    #     print("Error: result is None!")

    for i in range(beginLine, len(lines)):
        # if lines[i].point is None:
        #     print(f"Error: lines[{i}].point is None!")

        if det(lines[i].direction, lines[i].point - result) > distance:
            # 结果不满足约束 line i
            projLines = lines[:numObstLines]
            for j in range(numObstLines, i):
                line = Line()
                determinant = det(lines[i].direction, lines[j].direction)

                if abs(determinant) <= 1e-6:
                    # line i 和 line j 平行
                    if lines[i].direction * lines[j].direction > 0.0:
                        continue  # 两条线朝同一方向
                    line.point = 0.5 * (lines[i].point + lines[j].point)  # 朝反方向
                else:
                    line.point = lines[i].point + (
                            det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[
                                     i].direction

                line.direction = normalize(lines[j].direction - lines[i].direction)
                projLines.append(line)

            tempResult = copy.copy(result)
            len_line, result = linearProgram2(projLines, radius,
                                          Vector2(-lines[i].direction.y_, lines[i].direction.x_), True, result)
            if len_line < len(projLines):
                result = tempResult

            distance = det(lines[i].direction, lines[i].point - result)

    return result
