import numpy as np
import random
import math
import sys

class Map(object):
    def __init__(self, neuronNum, state_space, trialsNum):
        self.state_space = state_space
        self.trialsNum = trialsNum
        self.itercnt = 0

        self.mapRadius = max([math.fabs(r_max - r_min) for r_min, r_max in state_space])/2
        self.timeConst = self.trialsNum/math.log(self.mapRadius, 2)

        map = []
        for i in xrange(neuronNum):
            map.append(Map.selectRandomVector(state_space))

        self.map = np.array(map)
        #Seed 0
        random.seed(0)

    def getBestMatchingUnit(self, v):
        smallestDist = sys.maxint
        unit = self.map[0]
        #TODO: parallelize with torch
        for cell in self.map:
            dist = Map.getDist(v, cell)
            if dist < smallestDist:
                smallestDist = dist
                unit = cell

        return unit

    def epoch(self, sampled_vector, bmu, reward):
        neighbourhoodRadius = self.mapRadius * math.exp(-self.trialsNum/self.timeConst)
        learningRate = 1 * math.exp(self.itercnt/self.trialsNum)
        #TODO: parallelize with torch
        for i in xrange(len(self.map)):
            cell = self.map[i]
            distToCellSq = Map.getDist(cell, bmu)**2
            widthSq = neighbourhoodRadius**2

            if distToCellSq < widthSq:
                influence = math.exp(-distToCellSq/(2*widthSq))
                self.map[i] = np.add(learningRate * influence * self.getRewardCoef(self.itercnt, reward) * np.subtract(sampled_vector, cell), cell)

        self.itercnt += 1


    def getRewardCoef(self, k, efficiency):
        return 1

    @staticmethod
    def selectRandomVector(state_space):
        return [random.uniform(r_min,r_max) for r_min, r_max in state_space]

    @staticmethod
    def getDist(v1, v2):
        return math.sqrt(np.sum(np.square(np.subtract(v1, v2))))

class RewardMap(Map):
    def __init__(self, neuronNum, state_space, trialsNum, rho_min, rho_max):
        self.rho_min = rho_min
        self.rho_max = rho_max
        self.reward_min = 0
        self.reward_max = 1

        super(RewardMap, self).__init__(neuronNum, state_space, trialsNum)

    def getRewardCoef(self, k, reward):
        if k == 0:
            return self.rho_max
        else:
            if reward > self.reward_max:
                self.reward_max = reward
            elif reward < self.reward_min:
                self.reward_min = reward

            return (reward - self.reward_min)/(self.reward_max - self.reward_min) * (self.rho_max - self.rho_min) + self.rho_min

