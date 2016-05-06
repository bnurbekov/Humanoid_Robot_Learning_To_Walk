from map import RewardMap
from map import Map
import numpy as np
import math
import matlab.engine
import rospy
from std_msgs.msg import Float32, Empty
import os, os.path
import cma
import cPickle as pickle

class RobotInterface:
    def __init__(self):
        self.eng = matlab.engine.connect_matlab()
        self.success = None
        self.reward = None
        self.logFilename = "learning.log"
        self.logFile = open(self.logFilename, "a+")
        self.readingFromLogFirst = True

        #ROS publishers and subscribers
        self.rewardTopicName = "/sim_perf"

    def runTrial(self, params):
        print "Running trial with params: ", params

        if self.readingFromLogFirst:
            readResult = self.logFile.readline()
            if readResult:
                splitList = readResult.split(";")
                # previousParams = [i for i in splitList[0].split(",")]
                # if not numpy.isclose(params, previousParams):
                #     raise Exception("Params of the previous and current iterations are not identical!")
                sim_reward_out = float(splitList[1])
            else:
                self.logFile.seek(0)
                self.readingFromLogFirst = False

        if not self.readingFromLogFirst:
            self.eng.initSim(matlab.double(params))
            print "Inited sim."
            rospy.wait_for_message("/reset_done", Empty)
            print "Received reset confirmation."
            self.eng.resumeSim()
            print "Resumed sim."
            sim_reward_out = rospy.wait_for_message(self.rewardTopicName, Float32).data
            self.logFile.write(",".join((repr(x) for x in params)) + ";" + repr(sim_reward_out) + "\n")
            self.logFile.truncate()
            print "Received reward: ", sim_reward_out

        if sim_reward_out > 0:
            self.success = True
            self.reward = sim_reward_out
        else:
            self.success = False
            self.reward = 10*math.fabs(sim_reward_out)

        return self.success

    def getLogList(self):
        readResult = self.logFile.readline()
        while readResult:
            yield readResult.split(";")
            readResult = self.logFile.readline()

    def getReward(self):
        return self.reward

    def __del__(self):
        self.logFile.close()

class Learning:
    def __init__(self):
        self.simulation = RobotInterface()
        self.s_vig = 0
        self.vig_step = 0.01
        #Number of successful and failed trials
        self.n_s = 0
        self.n_f = 0
        #Initialize success and failure maps
        self.state_space = np.array([[0, 1], [0, 1], [0, 1], [0, 1]])
        self.map_s = RewardMap(625, self.state_space, 500, 0.1, 2.5)
        self.map_f = RewardMap(625, self.state_space, 500, 0.1, 2.5)

        self.converged = False

        #Settings
        self.runQARLNext = False

    def getVigThreshold(self):
        if self.n_s > self.n_f:
            self.s_vig -= self.vig_step
        elif self.n_f > self.n_s:
            self.s_vig += self.vig_step

    def run(self):
        #Test if maps have converged
        while not self.converged:
            if self.runQARLNext:
                self.runQARL()
            else:
                self.runCMA()

    def runCMA(self):
        # self.simulation.readingFromLogFirst = False
        #
        # # Restore the previous run on crush
        # if os.path.isfile("save.es") and os.stat("save.es").st_size != 0:
        #     es = pickle.load(open("save.es", "rb"))
        # else:
        #     es = cma.CMAEvolutionStrategy([0.450563106631,0.660245378622,0.996257839354,0.916941217947], 0.5, {'seed':807110})
        #
        # if os.path.isfile("cmaAns") and os.stat("cmaAns").st_size != 0:
        #     prevSolutions = pickle.load(open("cmaSols", "rb"))
        #     answers = pickle.load(open("cmaAns", "rb"))
        #
        #     solutions = es.ask()
        #
        #     if np.array_equal(prevSolutions[0], solutions[0]):
        #         self.cmaLearn(es, solutions, answers)
        #         answers = []
        # else:
        #     answers = []
        #
        # cmaSols = open("cmaSols", "wb")
        # cmaAns = open("cmaAns", "wb")

        es = cma.CMAEvolutionStrategy([0.450563106631,0.660245378622,0.996257839354,0.916941217947], 0.5, {'seed':807110, 'bounds': [[dim[0] for dim in self.state_space], [dim[1] for dim in self.state_space]]})

        while not es.stop() or not self.runQARLNext:
            print "CMA started new batch..."

            solutions = es.ask()

            answers = []
            for solution in solutions:
                solutionsList = solution.tolist()
                self.runQARLNext = self.simulation.runTrial(solutionsList)
                answers.append(self.simulation.getReward())

                bmu_f = self.map_f.getBestMatchingUnit(solutionsList)
                self.map_f.epoch(solutionsList, bmu_f, self.simulation.getReward())

            self.cmaLearn(es, solutions, answers)

        #es.result_pretty()
        self.runQARLNext = True
        # cmaSols.close()
        # cmaAns.close()

    def cmaLearn(self, es, solutions, answers):
        es.tell(solutions, answers)
        es.logger.add()  # write data to disc to be plotted
        #es.disp()
        # pickle.dump(es, open("save.es", "wb"))
        # pickle.dump(self.map_f, open("save.f_som", "wb"))

    def runQARL(self):
        v = Map.selectRandomVector(self.state_space)

        #Calculate a distance of this vector with each map
        bmu_s = self.map_s.getBestMatchingUnit(v)
        d_s = Map.getDist(v, bmu_s)
        bmu_f = self.map_f.getBestMatchingUnit(v)
        d_f = Map.getDist(v, bmu_f)

        #Vigilance adaptation
        self.s_vig = self.getVigThreshold()

        #Decision go/no-go
        if (d_f - d_s) > self.s_vig:
            isSuccess = self.simulation.runTrial(v)

            if isSuccess:
                self.map_s.epoch(v, bmu_s, self.simulation.getReward())
            else:
                self.map_f.epoch(v, bmu_f, self.simulation.getReward())

if __name__ == '__main__':
    # os.environ["ROS_MASTER_URI"] = "http://130.215.75.131:11311"
    # os.environ["ROS_IP"] = "130.215.79.147"
    rospy.init_node("learning")
    learning = Learning()
    #print sorted(learning.simulation.getLogList(), key=lambda x: float(x[1]))
    raw_input("Press Enter to continue...")
    learning.run()
