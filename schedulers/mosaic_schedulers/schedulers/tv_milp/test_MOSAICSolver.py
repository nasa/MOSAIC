"""
 Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED.
 United  States  Government  sponsorship  acknowledged.   Any commercial use
 must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the
 California Institute of Technology.
 
 This software may be subject to  U.S. export control laws  and regulations.
 By accepting this document,  the user agrees to comply  with all applicable
 U.S. export laws and regulations.  User  has the responsibility  to  obtain
 export  licenses,  or  other  export  authority  as may be required  before
 exporting  such  information  to  foreign  countries or providing access to
 foreign persons.
 
 This  software  is a copy  and  may not be current.  The latest  version is
 maintained by and may be obtained from the Mobility  and  Robotics  Sytstem
 Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches
 are welcome and should be sent to the software's maintainer.
 
"""

#import MOSAICSolver
import mosaic_schedulers.schedulers.tv_milp as MOSAICSolver
import unittest
import time
import copy
import json
try:
    import cplex  # This will be problematic on ARM
    CPLEXAvailable = True
except:
    CPLEXAvailable = False
try:
    import pyscipopt as scip
    SCIPAvailable = True
except:
    SCIPAvailable = False


def MosaicMILPWrapper(Thor,
                      AgentCapabilities,
                      Tasks,
                      CommNetwork,
                      Options={},
                      SolveMILP=True,
                      ):
    """" Used for simplicity in the tests. Unstable API. Do not use - you probably want to use JSONSolver instead. """
    # Default options
    if 'Solver' not in Options.keys():
        try:
            import cplex
            Options['Solver'] = 'CPLEX'
        except ImportError:
            Options['Solver'] = 'PuLP'

    TimeStep = 0.5
    Thor_s = float(Thor)*float(TimeStep)

    JSONinput = {}
    # Timing
    JSONinput["Time"] = {
        "Thor": Thor_s,
        "TimeStep": TimeStep
    }
    # Thor = int(float(Thor_s) / float(TimeStep))
    # Tasks
    JSONinput["Tasks"] = {
        "OptionalTasks": Tasks.OptionalTasks,
        "TaskReward": Tasks.TaskReward,
        "ProductsSize": Tasks.ProductsSize,
        "DependencyList": Tasks.DependencyList,
        "IncompatibleTasks": Tasks.IncompatibleTasks,

    }

    AllComputationTime = copy.deepcopy(AgentCapabilities.ComputationTime)
    for task, val in AllComputationTime.items():
        for agent, tasktime in val.items():
            AllComputationTime[task][agent] = float(tasktime) * float(TimeStep)

    assert len(list(AgentCapabilities.ComputationTime.keys())
               ) > 0, "ERROR: no tasks in ComputationTime"
    a_task_name = list(AgentCapabilities.ComputationTime.keys())[0]
    agent_names = list(AgentCapabilities.ComputationTime[a_task_name].keys())
    MaxComputationLoad = {agent: 1. for agent in agent_names}
    # Agent capabilities
    JSONinput["AgentCapabilities"] = {
        "ComputationTime": AllComputationTime,
        "ComputationLoad": AgentCapabilities.ComputationLoad,
        "InitialInformation": AgentCapabilities.InitialInformation,
        "EnergyCost": AgentCapabilities.EnergyCost,
        "MaxComputationLoad": MaxComputationLoad,
    }

    # Options
    JSONinput["Options"] = {
        "Packet Comms": Options.get("Packet Comms", False),
    }
    JSONinput["BandwidthMode"] = Options.get("BandwidthMode", "time_varying")

    if "CostFunction" in Options.keys():
        JSONinput["CostFunction"] = Options["CostFunction"]

    # Communication Windows
    CommunicationNetwork = []
    for t in CommNetwork.CommWindowsBandwidth.keys():
        for i in CommNetwork.CommWindowsBandwidth[t].keys():
            for j in CommNetwork.CommWindowsBandwidth[t][i].keys():
                if CommNetwork.CommWindowsBandwidth[t][i][j] > 0:
                    new_link = {
                        "time_start": t*TimeStep,
                        "time_end": (t+1)*TimeStep,
                        "info_time": 0.,
                        "origin": i,
                        "destination": j,
                        # In the struct format, bandwidth is the overall amount of data that can
                        # ben transferred in a window. In the JSON format, it is the amount of
                        # data per unit time. So we divide.
                        "bandwidth": float(CommNetwork.CommWindowsBandwidth[t][i][j])/float(TimeStep),
                        "energy_cost": CommNetwork.CommEnergyCost[t][i][j],
                    }
                    CommunicationNetwork.append(new_link)
    JSONinput["CommunicationNetwork"] = CommunicationNetwork

    JSONinput = json.dumps(JSONinput)

    # done initializing JSON

    _JSONsolver = MOSAICSolver.JSONSolver(
        JSONProblemDescription=JSONinput,
        solver=Options['Solver'],
        Verbose=False,
    )
    _JSONsolver.schedule()
    TaskAssignment, CommSchedule, RawOutput = _JSONsolver.Scheduler.getRawOutput()

    # if 'Packet Comms' not in Options.keys():
    #     Options['Packet Comms'] = False

    # if Options['Solver'] == 'CPLEX':
    #     # print("CPLEX!")
    #     _scheduler = MOSAICCPLEXScheduler(Thor,
    #                                       AgentCapabilities,
    #                                       Tasks,
    #                                       CommNetwork,
    #                                       TimeStep,
    #                                       Options)
    # elif Options['Solver'] == 'PuLP':
    #     # print("PuLP!")
    #     _scheduler = MOSAICPULPScheduler(Thor,
    #                                      AgentCapabilities,
    #                                      Tasks,
    #                                      CommNetwork,
    #                                      TimeStep,
    #                                      Options)

    # elif Options['Solver'] == 'GLPK':
    #     # print("GLPK!")
    #     _scheduler = MOSAICGLPKScheduler(Thor,
    #                                      AgentCapabilities,
    #                                      Tasks,
    #                                      CommNetwork,
    #                                      TimeStep,
    #                                      Options)
    # elif Options['Solver'] == 'SCIP':
    #     # print("SCIP!")
    #     _scheduler = MOSAICSCIPScheduler(Thor,
    #                                      AgentCapabilities,
    #                                      Tasks,
    #                                      CommNetwork,
    #                                      TimeStep,
    #                                      Options)
    # else:
    #     print("ERROR: please specify a valid solver ('CPLEX', 'PuLP', 'SCIP', or 'GLPK')")
    #     return -1

    # _scheduler.setUp()
    # _scheduler.solve()
    # _scheduler.formatToJSON()
    # TaskAssignment, CommSchedule, RawOutput = _scheduler.getRawOutput()

    return TaskAssignment, CommSchedule, RawOutput


class TestMILPScheduler(unittest.TestCase):
    def setUp(self):
        self.N = 2
        self.M = 2
        self.Thor = 2

    def initializeVars(self, N, M, Thor):
        AgentsList = []
        TasksList = []
        for i in range(N):
            AgentsList.append("Agent {}".format(i))
        for m in range(M):
            TasksList.append("Task {}".format(m))

        MyTasks = MOSAICSolver.MILPTasks()
        MyTasks.OptionalTasks.clear()
        MyTasks.TaskReward.clear()
        MyTasks.ProductsSize.clear()
        MyTasks.DependencyList.clear()
        for m in TasksList:
            MyTasks.OptionalTasks[m] = False
            MyTasks.TaskReward[m] = 1
            MyTasks.ProductsSize[m] = 1
            MyTasks.DependencyList[m] = []

        MyAgentCap = MOSAICSolver.MILPAgentCapabilities()
        MyAgentCap.ComputationTime.clear()
        MyAgentCap.ComputationLoad.clear()
        MyAgentCap.InitialInformation.clear()
        MyAgentCap.EnergyCost.clear()

        for m in TasksList:
            MyAgentCap.ComputationTime[m] = {}
            MyAgentCap.ComputationLoad[m] = {}
            MyAgentCap.InitialInformation[m] = {}
            MyAgentCap.EnergyCost[m] = {}
            for i in AgentsList:
                MyAgentCap.ComputationTime[m][i] = 1
                MyAgentCap.ComputationLoad[m][i] = 1
                MyAgentCap.InitialInformation[m][i] = False
                MyAgentCap.EnergyCost[m][i] = 0.1

        CommNetwork = MOSAICSolver.CommunicationNetwork()
        CommNetwork.CommWindowsBandwidth.clear()
        CommNetwork.CommEnergyCost.clear()
        CommWindowsBandwidth = {}
        CommEnergyCost = {}
        for t in range(Thor):
            CommWindowsBandwidth[t] = {}
            CommEnergyCost[t] = {}
            for i in AgentsList:
                CommWindowsBandwidth[t][i] = {}
                CommEnergyCost[t][i] = {}
                for j in AgentsList:
                    CommWindowsBandwidth[t][i][j] = 0
                    CommEnergyCost[t][i][j] = 0
        CommNetwork.CommWindowsBandwidth = CommWindowsBandwidth
        CommNetwork.CommEnergyCost = CommEnergyCost

        Options = {}
        Options["CostFunction"] = {"energy": 0.0,
                                   "total_task_reward": 1.0, "total_time": 0.0}
        return AgentsList, TasksList, MyAgentCap, MyTasks, CommNetwork, Options

    def available_solvers(self):
        solvers = ['GLPK', 'PuLP']
        if CPLEXAvailable:
            solvers.append('CPLEX')
        if SCIPAvailable:
            solvers.append('SCIP')
        return solvers

    def test_independent(self):

        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, _ = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 1'], 0)
            self.assertTrue(
                TaskAssignment['TaskAgent']['Task 0'] != TaskAssignment['TaskAgent']['Task 1'])

    def test_dependent(self):
        self.Thor = 3
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        MyTasks.DependencyList['Task 1'] = [['Task 0']]

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, _ = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 1'], 1)
            self.assertEqual(
                TaskAssignment['TaskAgent']['Task 0'], TaskAssignment['TaskAgent']['Task 1'])

    def test_communication(self):
        self.Thor = 4
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)

        for i in AgentsList:
            MyAgentCapabilities.ComputationTime['Task 0'][i] = self.Thor+1
            MyAgentCapabilities.ComputationTime['Task 1'][i] = self.Thor+1
        MyAgentCapabilities.ComputationTime['Task 0']['Agent 0'] = 1
        MyAgentCapabilities.ComputationTime['Task 1']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[1]['Agent 0']['Agent 1'] = 1
        MyTasks.DependencyList['Task 1'] = [['Task 0']]

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, _ = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 1'], 2)
            self.assertEqual(CommSchedule[0][0], 'Agent 0')
            self.assertEqual(CommSchedule[0][1], 'Agent 1')
            self.assertEqual(CommSchedule[0][2], 'Task 0')
            self.assertEqual(CommSchedule[0][3], 1)
            self.assertEqual(TaskAssignment['TaskAgent']['Task 0'], 'Agent 0')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 1'], 'Agent 1')

    def test_communication_large_products(self):
        self.Thor = 5
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)

        for i in AgentsList:
            MyAgentCapabilities.ComputationTime['Task 0'][i] = self.Thor+1
            MyAgentCapabilities.ComputationTime['Task 1'][i] = self.Thor+1
        MyAgentCapabilities.ComputationTime['Task 0']['Agent 0'] = 1
        MyAgentCapabilities.ComputationTime['Task 1']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[1]['Agent 0']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[2]['Agent 0']['Agent 1'] = 1
        MyTasks.DependencyList['Task 1'] = [['Task 0']]
        MyTasks.ProductsSize['Task 0'] = 2

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, _ = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 1'], 3)
            self.assertEqual(CommSchedule[0][0], 'Agent 0')
            self.assertEqual(CommSchedule[0][1], 'Agent 1')
            self.assertEqual(CommSchedule[0][2], 'Task 0')
            self.assertEqual(CommSchedule[0][3], 1)
            self.assertEqual(TaskAssignment['TaskAgent']['Task 0'], 'Agent 0')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 1'], 'Agent 1')

    def test_communication_large_products_delayed(self):
        self.Thor = 6
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)

        for i in AgentsList:
            MyAgentCapabilities.ComputationTime['Task 0'][i] = self.Thor+1
            MyAgentCapabilities.ComputationTime['Task 1'][i] = self.Thor+1
        MyAgentCapabilities.ComputationTime['Task 0']['Agent 0'] = 1
        MyAgentCapabilities.ComputationTime['Task 1']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[1]['Agent 0']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[3]['Agent 0']['Agent 1'] = 1
        MyTasks.DependencyList['Task 1'] = [['Task 0']]
        MyTasks.ProductsSize['Task 0'] = 2

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, _ = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 1'], 4)
            self.assertEqual(CommSchedule[0][0], 'Agent 0')
            self.assertEqual(CommSchedule[0][1], 'Agent 1')
            self.assertEqual(CommSchedule[0][2], 'Task 0')
            self.assertEqual(CommSchedule[0][3], 1)
            self.assertEqual(TaskAssignment['TaskAgent']['Task 0'], 'Agent 0')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 1'], 'Agent 1')

    def test_two_way_communication(self):
        self.Thor = 6
        self.M = 3

        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        for i in AgentsList:
            for m in TasksList:
                MyAgentCapabilities.ComputationTime[m][i] = self.Thor+1
        MyAgentCapabilities.ComputationTime['Task 0']['Agent 0'] = 1
        MyAgentCapabilities.ComputationTime['Task 1']['Agent 1'] = 1
        MyAgentCapabilities.ComputationTime['Task 2']['Agent 0'] = 1

        CommNetwork.CommWindowsBandwidth[1]['Agent 0']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[3]['Agent 1']['Agent 0'] = 1
        MyTasks.DependencyList['Task 1'] = [['Task 0']]
        MyTasks.DependencyList['Task 2'] = [['Task 1']]

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, _ = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            # Tasks are done sequentially
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 1'], 2)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 2'], 4)
            # Communication happens as expected: 1 to 2 about task 1 at time 2, 2 to 1
            # about task 2 at time 4
            self.assertEqual(CommSchedule[0][0], 'Agent 0')
            self.assertEqual(CommSchedule[0][1], 'Agent 1')
            self.assertEqual(CommSchedule[0][2], 'Task 0')
            self.assertEqual(CommSchedule[0][3], 1)
            self.assertEqual(CommSchedule[1][0], 'Agent 1')
            self.assertEqual(CommSchedule[1][1], 'Agent 0')
            self.assertEqual(CommSchedule[1][2], 'Task 1')
            self.assertEqual(CommSchedule[1][3], 3)
            self.assertEqual(TaskAssignment['TaskAgent']['Task 0'], 'Agent 0')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 1'], 'Agent 1')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 2'], 'Agent 0')

    def test_optional_tasks(self):
        self.N = 2
        self.M = 3
        self.Thor = 6
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        # Now this task is optional
        MyTasks.OptionalTasks['Task 2'] = True
        MyTasks.TaskReward['Task 2'] = 2
        MyTasks.DependencyList['Task 1'] = [['Task 0']]
        MyTasks.DependencyList['Task 2'] = [['Task 1']]

        for i in AgentsList:
            for m in TasksList:
                MyAgentCapabilities.ComputationTime[m][i] = self.Thor+1

        MyAgentCapabilities.ComputationTime['Task 0']['Agent 0'] = 1
        MyAgentCapabilities.ComputationTime['Task 1']['Agent 1'] = 1
        MyAgentCapabilities.ComputationTime['Task 2']['Agent 0'] = 1

        CommNetwork.CommWindowsBandwidth[1]['Agent 0']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[3]['Agent 1']['Agent 0'] = 1

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, _ = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            # Tasks are done sequentially
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 1'], 2)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 2'], 4)
            # Communication happens as expected: 1 to 2 about task 1 at time 2, 2 to 1
            # about task 2 at time 4
            self.assertEqual(CommSchedule[0][0], 'Agent 0')
            self.assertEqual(CommSchedule[0][1], 'Agent 1')
            self.assertEqual(CommSchedule[0][2], 'Task 0')
            self.assertEqual(CommSchedule[0][3], 1)
            self.assertEqual(CommSchedule[1][0], 'Agent 1')
            self.assertEqual(CommSchedule[1][1], 'Agent 0')
            self.assertEqual(CommSchedule[1][2], 'Task 1')
            self.assertEqual(CommSchedule[1][3], 3)
            self.assertEqual(TaskAssignment['TaskAgent']['Task 0'], 'Agent 0')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 1'], 'Agent 1')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 2'], 'Agent 0')

    def test_optional_tasks_skipped(self):
        self.N = 2
        self.M = 3
        self.Thor = 6

        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        # Now this task is optional
        MyTasks.OptionalTasks['Task 2'] = True

        for i in AgentsList:
            for m in TasksList:
                MyAgentCapabilities.ComputationTime[m][i] = self.Thor+1

        MyAgentCapabilities.ComputationTime['Task 0']['Agent 0'] = 1
        MyAgentCapabilities.ComputationTime['Task 1']['Agent 1'] = 1
        MyAgentCapabilities.ComputationTime['Task 2']['Agent 0'] = 1

        MyTasks.DependencyList['Task 1'] = [['Task 0']]
        MyTasks.DependencyList['Task 2'] = [['Task 1']]
        CommNetwork.CommWindowsBandwidth[1]['Agent 0']['Agent 1'] = 1
        # No communication for you: the next line is commented
        # self.CommWindowsBandwidth[3][1][0] = 1
        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, _ = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            # Tasks are done sequentially, task 3 is skipped
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertTrue(TaskAssignment['TaskSchedule']['Task 1'] >= 2)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 2'], None)
            # Communication happens as expected: 1 to 2 about task 1 at time 2
            self.assertEqual(CommSchedule[0][0], 'Agent 0')
            self.assertEqual(CommSchedule[0][1], 'Agent 1')
            self.assertEqual(CommSchedule[0][2], 'Task 0')
            self.assertEqual(CommSchedule[0][3], 1)
            # Agents split up the load, task 3 is skipped
            self.assertEqual(TaskAssignment['TaskAgent']['Task 0'], 'Agent 0')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 1'], 'Agent 1')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 2'], None)

    def test_or_reqs(self):
        self.N = 2
        self.M = 3
        self.Thor = 4

        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        # Now this task is optional
        MyTasks.OptionalTasks['Task 2'] = True

        for i in AgentsList:
            for m in TasksList:
                MyAgentCapabilities.ComputationTime[m][i] = self.Thor+1
        MyAgentCapabilities.ComputationTime['Task 0']['Agent 0'] = 1
        MyAgentCapabilities.ComputationTime['Task 1']['Agent 1'] = 1
        # Now you can't do Task 2 ever

        # Either 0 or 2 must be done for 1 to be done
        MyTasks.DependencyList['Task 1'] = [['Task 0', 'Task 2']]
        CommNetwork.CommWindowsBandwidth[1]['Agent 0']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[3]['Agent 1']['Agent 0'] = 1

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, RawOutput = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            # Tasks are done sequentially
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 1'], 2)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 2'], None)
            # Communication happens as expected: 1 to 2 about task 1 at time 2
            self.assertEqual(CommSchedule[0][0], 'Agent 0')
            self.assertEqual(CommSchedule[0][1], 'Agent 1')
            self.assertEqual(CommSchedule[0][2], 'Task 0')
            self.assertEqual(CommSchedule[0][3], 1)
            # Agents split up the load, task 3 is skipped
            self.assertEqual(TaskAssignment['TaskAgent']['Task 0'], 'Agent 0')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 1'], 'Agent 1')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 2'], None)

    def test_impossible_reqs(self):
        self.N = 2
        self.M = 3
        self.Thor = 4
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        MyTasks.OptionalTasks['Task 2'] = True
        for i in AgentsList:
            for m in TasksList:
                MyAgentCapabilities.ComputationTime[m][i] = self.Thor+1
        MyAgentCapabilities.ComputationTime['Task 0']['Agent 0'] = 1
        MyAgentCapabilities.ComputationTime['Task 1']['Agent 1'] = 1
        # Now you can't do Task 2 ever

        # BOTH 0 AND 2 must be done for 1 to be done
        MyTasks.DependencyList['Task 1'] = [['Task 0'], ['Task 2']]
        CommNetwork.CommWindowsBandwidth[1]['Agent 0']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[3]['Agent 1']['Agent 0'] = 1

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, RawOutput = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], None)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 1'], None)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 2'], None)

    def test_chain_of_reqs(self):
        self.N = 5
        self.M = 5
        self.Thor = 10
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        for i in AgentsList:
            for m in TasksList:
                MyAgentCapabilities.ComputationTime[m][i] = self.Thor+1

        for i in range(self.N):
            MyAgentCapabilities.ComputationTime['Task {}'.format(
                i)]['Agent {}'.format(i)] = 1

        for m in range(1, len(TasksList)):
            MyTasks.DependencyList[TasksList[m]] = [[TasksList[m-1]]]

        for i in range(self.N-1):
            CommNetwork.CommWindowsBandwidth[2*i +
                                             1]['Agent {}'.format(i)]['Agent {}'.format(i+1)] = 1
            CommNetwork.CommEnergyCost[2*i +
                                       1]['Agent {}'.format(i)]['Agent {}'.format(i+1)] = 0.1

        for solver in self.available_solvers():
            Options['Solver'] = solver
            startTime = time.time()
            TaskAssignment, CommSchedule, RawOutput = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            endTime = time.time() - startTime
            print("Time to solve: {}".format(endTime))
            for i in range(self.N):
                self.assertEqual(
                    TaskAssignment['TaskAgent']['Task {}'.format(i)], 'Agent {}'.format(i))
                self.assertEqual(
                    TaskAssignment['TaskSchedule']['Task {}'.format(i)], 2*i)

    def test_chain_of_reqs_broken_comms(self):
        self.N = 5
        self.M = 5
        self.Thor = 10
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)

        for i in AgentsList:
            for m in TasksList:
                MyAgentCapabilities.ComputationTime[m][i] = self.Thor+1

        for i in range(self.N):
            MyAgentCapabilities.ComputationTime['Task {}'.format(
                i)]['Agent {}'.format(i)] = 1

        for m in range(1, len(TasksList)):
            MyTasks.DependencyList[TasksList[m]] = [[TasksList[m-1]]]

        for i in range(self.N-2):
            CommNetwork.CommWindowsBandwidth[2*i +
                                             1]['Agent {}'.format(i)]['Agent {}'.format(i+1)] = 1

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, RawOutput = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            for i in range(self.N):
                self.assertEqual(
                    TaskAssignment['TaskAgent']['Task {}'.format(i)], None)
                self.assertEqual(
                    TaskAssignment['TaskSchedule']['Task {}'.format(i)], None)

    def test_chain_of_reqs_broken_comms_optional_task(self):
        self.N = 5
        self.M = 5
        self.Thor = 10
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        MyTasks.OptionalTasks['Task {}'.format(self.M-1)] = True

        for i in AgentsList:
            for m in TasksList:
                MyAgentCapabilities.ComputationTime[m][i] = self.Thor+1

        for i in range(self.N):
            MyAgentCapabilities.ComputationTime['Task {}'.format(
                i)]['Agent {}'.format(i)] = 1

        for m in range(1, len(TasksList)):
            MyTasks.DependencyList[TasksList[m]] = [[TasksList[m-1]]]

        for i in range(self.N - 2):
            CommNetwork.CommWindowsBandwidth[2 * i +
                                             1]['Agent {}'.format(i)]['Agent {}'.format(i + 1)] = 1

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, RawOutput = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            for i in range(self.M - 1):
                self.assertEqual(
                    TaskAssignment['TaskAgent']['Task {}'.format(i)], 'Agent {}'.format(i))
                self.assertTrue(
                    TaskAssignment['TaskSchedule']['Task {}'.format(i)] >= 2 * i)

    def test_optional_needed_for_required(self):
        """two optional tasks required for a mandatory task."""
        self.N = 1
        self.M = 4
        self.Thor = 4

        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        MyTasks.OptionalTasks['Task 1'] = True
        MyTasks.OptionalTasks['Task 2'] = True
        MyTasks.DependencyList['Task 1'] = [['Task 0']]
        MyTasks.DependencyList['Task 2'] = [['Task 0']]
        MyTasks.DependencyList['Task 3'] = [['Task 0'], ['Task 1', 'Task 2']]

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, RawOutput = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 3'], 2)
            self.assertTrue((TaskAssignment['TaskSchedule']['Task 1'] != None) or (
                TaskAssignment['TaskSchedule']['Task 2'] != None))

    def test_optional_needed_for_required_more_time(self):
        """two optional tasks required for a mandatory task, but time to do all."""
        self.N = 1
        self.M = 4
        self.Thor = 5
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        MyTasks.OptionalTasks['Task 1'] = True
        MyTasks.OptionalTasks['Task 2'] = True
        MyTasks.DependencyList['Task 1'] = [['Task 0']]
        MyTasks.DependencyList['Task 2'] = [['Task 0']]
        MyTasks.DependencyList['Task 3'] = [['Task 0'], ['Task 1'], ['Task 2']]

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, RawOutput = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)

            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 3'], 3)
            self.assertTrue(TaskAssignment['TaskSchedule']['Task 1'] != None)
            self.assertTrue(TaskAssignment['TaskSchedule']['Task 2'] != None)

    def test_incompatible_tasks_v1(self):
        """Incompatible tasks (base)."""
        self.N = 2
        self.M = 2
        self.Thor = 2
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        MyTasks.OptionalTasks['Task 1'] = True
        MyTasks.IncompatibleTasks = [['Task 0', 'Task 1']]

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, RawOutput = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)

            self.assertEqual(TaskAssignment['TaskSchedule']['Task 1'], None)
            self.assertEqual(TaskAssignment['TaskAgent']['Task 1'], None)
            self.assertEqual(TaskAssignment['TaskSchedule']['Task 0'], 0)
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] != None)

    def test_incompatible_tasks_v2(self):
        """Incompatible tasks (v2)."""
        self.N = 2
        self.M = 3
        self.Thor = 6
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        MyTasks.OptionalTasks['Task 2'] = True
        MyTasks.IncompatibleTasks = [['Task 1', 'Task 2']]
        for i in AgentsList:
            for m in TasksList:
                MyAgentCapabilities.ComputationTime[m][i] = self.Thor+1
        MyAgentCapabilities.ComputationTime['Task 0']['Agent 0'] = 1
        MyAgentCapabilities.ComputationTime['Task 1']['Agent 1'] = 1
        MyAgentCapabilities.ComputationTime['Task 2']['Agent 0'] = 1

        MyTasks.DependencyList['Task 1'] = [['Task 0']]
        MyTasks.DependencyList['Task 2'] = [['Task 1']]
        CommNetwork.CommWindowsBandwidth[1]['Agent 0']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[3]['Agent 1']['Agent 0'] = 1

        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, RawOutput = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)

            self.assertEqual(TaskAssignment['TaskSchedule']['Task 2'], None)
            self.assertEqual(TaskAssignment['TaskAgent']['Task 2'], None)

    def test_incompatible_tasks_v3(self):
        """Incompatible tasks (v3)."""
        self.N = 2
        self.M = 3
        self.Thor = 6
        AgentsList, TasksList, MyAgentCapabilities, MyTasks, CommNetwork, Options = self.initializeVars(
            self.N, self.M, self.Thor)
        MyTasks.OptionalTasks['Task 2'] = True
        MyTasks.IncompatibleTasks = [
            ['Task 0', 'Task 2'], ['Task 1', 'Task 2']]

        for i in AgentsList:
            for m in TasksList:
                MyAgentCapabilities.ComputationTime[m][i] = self.Thor + 1

        MyAgentCapabilities.ComputationTime['Task 0']['Agent 0'] = 1
        MyAgentCapabilities.ComputationTime['Task 1']['Agent 1'] = 1
        MyAgentCapabilities.ComputationTime['Task 2']['Agent 0'] = 1
        MyTasks.DependencyList['Task 1'] = [['Task 0']]
        MyTasks.DependencyList['Task 2'] = [['Task 1']]
        CommNetwork.CommWindowsBandwidth[1]['Agent 0']['Agent 1'] = 1
        CommNetwork.CommWindowsBandwidth[3]['Agent 1']['Agent 0'] = 1
        for solver in self.available_solvers():
            Options['Solver'] = solver
            TaskAssignment, CommSchedule, RawOutput = MosaicMILPWrapper(
                self.Thor, MyAgentCapabilities, MyTasks, CommNetwork, Options)

            self.assertEqual(TaskAssignment['TaskSchedule']['Task 2'], None)
            self.assertEqual(TaskAssignment['TaskAgent']['Task 2'], None)


if __name__ == '__main__':
    unittest.main()
