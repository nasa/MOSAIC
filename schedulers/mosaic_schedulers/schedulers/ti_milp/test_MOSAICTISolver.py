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

import mosaic_schedulers.schedulers.ti_milp as MOSAICTISolver
import unittest
import time
try:
    import cplex  # This will be problematic on ARM
    CPLEXAvailable = True
except:
    CPLEXAvailable = False


class TestMILPTIScheduler(unittest.TestCase):
    def setUp(self):
        self.N = 2
        self.M = 2
        self.Thor = 60

    def initializeVars(self, N, M):
        AgentsList = []
        TasksList = []
        for i in range(N):
            AgentsList.append("Agent {}".format(i))
        for m in range(M):
            TasksList.append("Task {}".format(m))

        MyTasks = MOSAICTISolver.MILPTasks()
        MyTasks.OptionalTasks.clear()
        MyTasks.TaskReward.clear()
        MyTasks.ProductsDataSize.clear()
        MyTasks.ProductsBandwidth.clear()
        MyTasks.DependencyList.clear()
        # MyTasks.IncompatibleTasks.clear()
        MyTasks.MaxLatency.clear()
        for m in TasksList:
            MyTasks.ProductsDataSize[m] = 1
            MyTasks.ProductsBandwidth[m] = 1
            MyTasks.DependencyList[m] = []
            MyTasks.OptionalTasks[m] = False
            MyTasks.TaskReward[m] = 0.
            MyTasks.MaxLatency[m] = {}

        MyAgentCap = MOSAICTISolver.MILPAgentCapabilities()
        MyAgentCap.ComputationLoad.clear()
        MyAgentCap.EnergyCost.clear()
        MyAgentCap.MaxComputationLoad.clear()
        MyAgentCap.LinkComputationalLoadIn.clear()
        MyAgentCap.LinkComputationalLoadOut.clear()

        for i in AgentsList:
            MyAgentCap.ComputationLoad[i] = {}
            MyAgentCap.EnergyCost[i] = {}
            for m in TasksList:
                MyAgentCap.ComputationLoad[i][m] = .5
                MyAgentCap.EnergyCost[i][m] = 1.
            MyAgentCap.MaxComputationLoad[i] = 1.
            MyAgentCap.LinkComputationalLoadIn[i] = {}
            MyAgentCap.LinkComputationalLoadOut[i] = {}
            for j in AgentsList:
                MyAgentCap.LinkComputationalLoadIn[i][j] = .1
                MyAgentCap.LinkComputationalLoadOut[i][j] = .1

        MyCommNet = MOSAICTISolver.CommunicationNetwork()
        MyCommNet.Bandwidth.clear()
        MyCommNet.EnergyCost.clear()
        MyCommNet.Latency.clear()
        for i in AgentsList:
            MyCommNet.Bandwidth[i] = {}
            MyCommNet.EnergyCost[i] = {}
            MyCommNet.Latency[i] = {}
            for j in AgentsList:
                MyCommNet.Bandwidth[i][j] = 1.
                MyCommNet.EnergyCost[i][j] = .2
                MyCommNet.Latency[i][j] = .0001

        Options = {
            "CostFunction":
            {
                "energy": 0.0,
                "total_task_reward": 1.0,
                "total_time": 0.0
            }
        }
        return AgentsList, TasksList, MyAgentCap, MyTasks, MyCommNet, Options

    def available_solvers(self):
        solvers = ['GLPK', 'SCIP']
        if CPLEXAvailable:
            solvers.append('CPLEX')
        return solvers

    def call_solver(self, MyAgentCap, MyTasks, MyCommNet, MyOptions, solver='GLPK'):
        # Extract list of tasks and of agents
        tasks = MyTasks.ProductsBandwidth.keys()
        agents = MyAgentCap.ComputationLoad.keys()

        # Use an arbitrary time horizon
        Thor = self.Thor

        JSONInput = {}
        # Time
        JSONInput["Time"] = {
            "Thor": Thor,
            "TimeStep": None  # Not used
        }

        # Tasks
        # The JSON file expects task dependencies in disjunctive form.
        # But the TI MILP does not support that yet.
        dep_list = {}
        for task in tasks:
            dep_list[task] = []
            if task in MyTasks.DependencyList.keys():
                if MyTasks.DependencyList[task]:
                    for predecessor in MyTasks.DependencyList[task]:
                        dep_list[task].append([predecessor, ])

        # The JSON reports actual data sizes, but the internal solver structure
        # uses instantaneous bandwidth instead
        pr_size = {}
        for task, size in MyTasks.ProductsBandwidth.items():
            pr_size[task] = float(size*Thor)

        JSONInput["Tasks"] = {
            "DependencyList": dep_list,
            "ProductsSize": pr_size,
            "OptionalTasks": MyTasks.OptionalTasks,
            "TaskReward": MyTasks.TaskReward,
            "MaxLatency": MyTasks.MaxLatency,
        }

        # Agent capabilities

        # Computation Load: defined as 1
        ComputationLoad = {task: {agent: 1. for agent in agents}
                           for task in tasks}

        # Computation Time: TI computation load * time horizon
        ComputationTime = {task: {
            agent: MyAgentCap.ComputationLoad[agent][task]*Thor for agent in agents} for task in tasks}

        TVLinkLoadIn = {agent1: {
            agent2: MyAgentCap.LinkComputationalLoadIn[agent1][agent2]*Thor for agent2 in agents} for agent1 in agents}
        TVLinkLoadOut = {agent1: {
            agent2: MyAgentCap.LinkComputationalLoadOut[agent1][agent2]*Thor for agent2 in agents} for agent1 in agents}

        EnergyCost = MOSAICTISolver.flipNestedDictionary(MyAgentCap.EnergyCost)

        JSONInput["AgentCapabilities"] = {
            "ComputationTime": ComputationTime,
            "ComputationLoad": ComputationLoad,
            "LinkComputationalLoadIn": TVLinkLoadIn,
            "LinkComputationalLoadOut": TVLinkLoadOut,
            "EnergyCost": EnergyCost,
            "MaxComputationLoad": MyAgentCap.MaxComputationLoad,
        }

        # Communication Network
        TVCommNet = []
        for agent1 in MyCommNet.Bandwidth.keys():
            for agent2 in MyCommNet.Bandwidth[agent1].keys():
                TVCommNet.append({
                    'info_time': 0.,
                    'origin': agent1,
                    'destination': agent2,
                    'time_start': 0.,
                    'time_end': Thor,
                    'bandwidth': MyCommNet.Bandwidth[agent1][agent2],
                    'energy_cost': MyCommNet.EnergyCost[agent1][agent2],
                    'latency': MyCommNet.Latency[agent1][agent2],
                })
        JSONInput["CommunicationNetwork"] = TVCommNet

        # Options
        JSONInput["Options"] = MyOptions

        # Instantiate the problem
        problem = MOSAICTISolver.JSONSolver(
            JSONProblemDescription=JSONInput,
            solver=solver,
        )

        # problem.schedule()
        return problem.Scheduler

        # if solver not in self.available_solvers():
        #     raise ValueError("Solver not supported on this machine")
        # if solver == 'GLPK':
        #     return MOSAICTISolver.MOSAICGLPKSolver(MyAgentCap, MyTasks, MyCommNet, MyOptions)
        # if solver == 'CPLEX':
        #     return MOSAICTISolver.MOSAICCPLEXSolver(MyAgentCap, MyTasks, MyCommNet, MyOptions)
        # if solver == 'SCIP':
        #     return MOSAICTISolver.MOSAICSCIPSolver(MyAgentCap, MyTasks, MyCommNet, MyOptions)

    def isInComms(self, message, CommSchedule, threshold=0.01):
        found = False
        for comm in CommSchedule:
            if message[:3] == comm[:3]:
                if abs(message[3]-comm[3]) < threshold:
                    found = True
                    break
        return found

    def test_onetask(self):
        '''One agent, one task: looking for syntax errors, etc.'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            1, 1)
        MyTasks.validate()
        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertEqual(TaskAssignment['TaskAgent']['Task 0'], 'Agent 0')

    def test_twotasks(self):
        '''One agent, two tasks: looking for syntax errors, etc.'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=1, M=2)
        MyTasks.validate()
        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertEqual(TaskAssignment['TaskAgent']['Task 0'], 'Agent 0')
            self.assertEqual(TaskAssignment['TaskAgent']['Task 1'], 'Agent 0')

    def test_twoagents(self):
        '''Two agents, one task: looking for syntax errors, etc.'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=2, M=1)
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = 1.
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 1.
        MyTasks.validate()
        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] ==
                            'Agent 0' or TaskAssignment['TaskAgent']['Task 0'] == 'Agent 1')

    def test_twoandtwo(self):
        '''Two agents, two task: looking for syntax errors, etc.'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=2, M=2)
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = 1.
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 1.
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = 1.
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = 1.
        MyTasks.validate()
        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] ==
                            'Agent 0' or TaskAssignment['TaskAgent']['Task 0'] == 'Agent 1')
            self.assertTrue(
                TaskAssignment['TaskAgent']['Task 0'] != TaskAssignment['TaskAgent']['Task 1'])

    def test_independent(self):
        '''Two agents, two task: only one agent can do each task.'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=2, M=2)
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = 1.
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = 1.
        MyTasks.DependencyList = {}
        MyTasks.MaxLatency = {}
        MyTasks.validate()
        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 1')

    def test_dependent(self):
        '''Two agents, two task: only one agent can do each task, and they must communicate.'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=2, M=2)
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .7
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = .7
        MyTasks.DependencyList['Task 1'] = ['Task 0']
        MyTasks.MaxLatency['Task 1']['Task 0'] = 999.
        MyTasks.validate()
        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, CommSchedule) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 1')
            # self.assertTrue(CommSchedule[0] == ['Agent 0', 'Agent 1', 'Task 0', 1.0])
            self.assertTrue(self.isInComms(
                ['Agent 0', 'Agent 1', 'Task 0', 1.0], CommSchedule))

    def test_two_way(self):
        ''' Agents must communicate back and forth to complete all tasks '''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=2, M=3)
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 2'] = 2.
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .3
        MyAgentCap.ComputationLoad['Agent 0']['Task 2'] = .3
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = .3

        MyTasks.DependencyList['Task 1'] = ['Task 0']
        MyTasks.DependencyList['Task 2'] = ['Task 1']
        MyTasks.MaxLatency['Task 1']['Task 0'] = 999.
        MyTasks.MaxLatency['Task 2']['Task 1'] = 999.
        MyTasks.validate()
        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, CommSchedule) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 1')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 2'] == 'Agent 0')
            #self.assertTrue(['Agent 0', 'Agent 1', 'Task 0', 1.0] in CommSchedule)
            #self.assertTrue(['Agent 1', 'Agent 0', 'Task 1', 1.0] in CommSchedule)
            self.assertTrue(self.isInComms(
                ['Agent 0', 'Agent 1', 'Task 0', 1.0], CommSchedule))
            self.assertTrue(self.isInComms(
                ['Agent 1', 'Agent 0', 'Task 1', 1.0], CommSchedule))

    def test_latency_infeasible(self):
        ''' we verify that if the latency constraint cannot be satisfied, the problem fails '''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=2, M=2)
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .7
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = .7
        MyTasks.DependencyList['Task 1'] = ['Task 0']
        MyTasks.MaxLatency['Task 1']['Task 0'] = .09
        # We will need extra time to deliver the data product
        MyCommNet.Latency['Agent 0']['Agent 1'] = .1
        MyTasks.validate()

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, CommSchedule) = Solver.schedule()
            self.assertTrue(TaskAssignment is None)
            self.assertTrue(CommSchedule is None)

    def test_latency_feasible(self):
        ''' we verify that if the latency constraint can be satisfied, the problem does find a solution '''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=2, M=2)
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .7
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = .7
        MyTasks.DependencyList['Task 1'] = ['Task 0']
        MyTasks.ProductsBandwidth['Task 0'] = .2
        MyCommNet.Bandwidth['Agent 0']['Agent 1'] = 1.*self.Thor
        MyCommNet.Latency['Agent 0']['Agent 1'] = .1
        MyTasks.MaxLatency['Task 1']['Task 0'] = .2 + MyTasks.ProductsBandwidth['Task 0'] * \
            self.Thor/MyCommNet.Bandwidth['Agent 0']['Agent 1']
        MyTasks.validate()

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, CommSchedule) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 1')
            self.assertTrue(self.isInComms(
                ['Agent 0', 'Agent 1', 'Task 0', 0.2], CommSchedule))

    def test_latency_choice(self):
        ''' Two agents can do a task. But only one can do it in time '''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=3, M=2)
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .7
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 2']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = .7
        MyAgentCap.ComputationLoad['Agent 2']['Task 1'] = .7

        MyTasks.DependencyList['Task 1'] = ['Task 0']
        MyTasks.ProductsBandwidth['Task 0'] = .1
        # ProductsDataSize is computed as ProductsBandwidth*self.Thor
        # So the latency introduced by the message size is PrBw*Thor/BWij
        MyCommNet.Bandwidth['Agent 0']['Agent 1'] = 1.*self.Thor
        MyCommNet.Bandwidth['Agent 0']['Agent 2'] = 1.*self.Thor
        # We will need extra time to deliver the data product
        MyCommNet.Latency['Agent 0']['Agent 1'] = .25
        # We will need extra time to deliver the data product
        MyCommNet.Latency['Agent 0']['Agent 2'] = .2
        # We do have a deadline, though
        MyTasks.MaxLatency['Task 1']['Task 0'] = .2+MyTasks.ProductsBandwidth['Task 0'] * \
            self.Thor/MyCommNet.Bandwidth['Agent 0']['Agent 2']

        MyTasks.validate()

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, CommSchedule) = Solver.schedule()
            self.assertTrue(TaskAssignment)
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 2')
            self.assertTrue(CommSchedule[0] == [
                            'Agent 0', 'Agent 2', 'Task 0', .1])

    def test_multiple_dependencies(self):
        # Task 2 depends on tasks 0 and 1
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=2, M=3)
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .4
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = .4
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 0']['Task 2'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 2'] = .8
        MyTasks.DependencyList['Task 2'] = ['Task 0', 'Task 1']
        MyTasks.ProductsBandwidth['Task 0'] = .1
        MyTasks.ProductsBandwidth['Task 1'] = .1
        MyCommNet.Bandwidth['Agent 0']['Agent 1'] = .2
        MyTasks.MaxLatency['Task 2']['Task 0'] = .2+MyTasks.ProductsBandwidth['Task 0'] * \
            self.Thor/MyCommNet.Bandwidth['Agent 0']['Agent 1']
        MyTasks.MaxLatency['Task 2']['Task 1'] = .2+MyTasks.ProductsBandwidth['Task 1'] * \
            self.Thor/MyCommNet.Bandwidth['Agent 0']['Agent 1']
        MyTasks.validate()

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, CommSchedule) = Solver.schedule()
            self.assertTrue(TaskAssignment)
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 2'] == 'Agent 1')
            self.assertTrue(self.isInComms(
                ['Agent 0', 'Agent 1', 'Task 0', .1], CommSchedule))
            self.assertTrue(self.isInComms(
                ['Agent 0', 'Agent 1', 'Task 1', .1], CommSchedule))

    def test_multi_hop(self):
        # Agent 0 does task 0, agent 2 does task 1, agent 1 acts as a relay
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=3, M=2)
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .4
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 2']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 2']['Task 1'] = .4
        MyTasks.DependencyList['Task 1'] = ['Task 0']
        MyCommNet.Bandwidth['Agent 0']['Agent 2'] = 0.
        MyCommNet.Bandwidth['Agent 2']['Agent 0'] = 0.
        MyTasks.MaxLatency['Task 1']['Task 0'] = .2 \
            + MyTasks.ProductsBandwidth['Task 0']*self.Thor / MyCommNet.Bandwidth['Agent 0']['Agent 1'] + \
            MyTasks.ProductsBandwidth['Task 0'] * self.Thor / \
            MyCommNet.Bandwidth['Agent 1']['Agent 2']
        MyTasks.validate()

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, CommSchedule) = Solver.schedule()
            self.assertTrue(TaskAssignment)
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 2')
            self.assertTrue(self.isInComms(
                ['Agent 0', 'Agent 1', 'Task 0', 1.], CommSchedule))
            self.assertTrue(self.isInComms(
                ['Agent 1', 'Agent 2', 'Task 0', 1.], CommSchedule))

    def test_multi_hop_and_back(self):
        # Agent 0 does tasks 0 and 2, agent 2 does task 1, agent 1 relays back and forth.
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=3, M=3)
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .3
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 0']['Task 2'] = .3
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = 2.
        MyAgentCap.ComputationLoad['Agent 1']['Task 2'] = 2.
        MyAgentCap.ComputationLoad['Agent 2']['Task 0'] = 2.
        MyAgentCap.ComputationLoad['Agent 2']['Task 1'] = .4
        MyAgentCap.ComputationLoad['Agent 2']['Task 2'] = 2.
        MyTasks.DependencyList['Task 1'] = ['Task 0']
        MyTasks.DependencyList['Task 2'] = ['Task 1']
        MyCommNet.Bandwidth['Agent 0']['Agent 2'] = 0.
        MyCommNet.Bandwidth['Agent 2']['Agent 0'] = 0.

        MyTasks.MaxLatency['Task 1']['Task 0'] = .2 \
            + MyTasks.ProductsBandwidth['Task 0']*self.Thor / MyCommNet.Bandwidth['Agent 0']['Agent 1'] + \
            MyTasks.ProductsBandwidth['Task 0'] * self.Thor / \
            MyCommNet.Bandwidth['Agent 1']['Agent 2']

        MyTasks.MaxLatency['Task 2']['Task 1'] = .2 \
            + MyTasks.ProductsBandwidth['Task 1']*self.Thor / MyCommNet.Bandwidth['Agent 2']['Agent 1'] + \
            MyTasks.ProductsBandwidth['Task 1'] * self.Thor / \
            MyCommNet.Bandwidth['Agent 1']['Agent 0']

        MyTasks.validate()

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, CommSchedule) = Solver.schedule()
            self.assertTrue(TaskAssignment)
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 2')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 2'] == 'Agent 0')
            self.assertTrue(self.isInComms(
                ['Agent 0', 'Agent 1', 'Task 0', 1.], CommSchedule))
            self.assertTrue(self.isInComms(
                ['Agent 1', 'Agent 2', 'Task 0', 1.], CommSchedule))
            self.assertTrue(self.isInComms(
                ['Agent 2', 'Agent 1', 'Task 1', 1.], CommSchedule))
            self.assertTrue(self.isInComms(
                ['Agent 1', 'Agent 0', 'Task 1', 1.], CommSchedule))

    def test_long_chain(self):
        n_agents = 10
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=n_agents, M=n_agents)
        for agent in range(n_agents - 1):
            MyTasks.DependencyList['Task {}'.format(
                agent+1)] = ['Task {}'.format(agent)]
            MyTasks.MaxLatency['Task {}'.format(agent+1)
                               ]['Task {}'.format(agent)] = 999.
        for agent in range(n_agents):
            for agent2 in range(n_agents):
                if agent2 != agent:
                    MyAgentCap.ComputationLoad['Agent {}'.format(
                        agent2)]['Task {}'.format(agent)] = 2.
        startTime = time.time()
        MyTasks.validate()

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            endTime = time.time() - startTime
            print("Time elapsed ({}): {}".format(_solver, endTime))
            for agent in range(n_agents):
                self.assertTrue(TaskAssignment['TaskAgent']['Task {}'.format(
                    agent)] == 'Agent {}'.format(agent))

    def test_optional(self):
        '''one agents, one optional task: looking for syntax errors, etc.'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=1, M=1)
        MyTasks.OptionalTasks['Task 0'] = True
        MyTasks.TaskReward['Task 0'] = 1.
        MyTasks.validate()
        MyOptions['cost'] = 'optional'

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')

    def test_optional_infeasible(self):
        '''one agents, one optional task and a required one'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=1, M=2)
        MyTasks.OptionalTasks['Task 0'] = False
        MyTasks.OptionalTasks['Task 1'] = True
        MyTasks.TaskReward['Task 0'] = 0.
        MyTasks.TaskReward['Task 1'] = 1.
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = 1.
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = 1.
        MyTasks.validate()
        MyOptions['cost'] = 'optional'

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] is None)

    def test_optional_feasible(self):
        '''one agents, one optional task and a required one'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=1, M=2)
        MyTasks.OptionalTasks['Task 0'] = False
        MyTasks.OptionalTasks['Task 1'] = True
        MyTasks.TaskReward['Task 0'] = 0.
        MyTasks.TaskReward['Task 1'] = 1.
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .5
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = .5
        MyTasks.validate()
        MyOptions['cost'] = 'optional'

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 0')

    def test_optional_required_for_feasible(self):
        '''one agents, one optional task and a required one'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=1, M=2)
        MyTasks.OptionalTasks['Task 0'] = False
        MyTasks.OptionalTasks['Task 1'] = True
        MyTasks.TaskReward['Task 0'] = 0.
        MyTasks.TaskReward['Task 1'] = 1.
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .5
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = .5
        MyTasks.DependencyList['Task 0'] = ['Task 1']
        MyTasks.MaxLatency['Task 0']['Task 1'] = 999.
        MyTasks.validate()
        MyOptions['cost'] = 'optional'

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 0')

    def test_mixed_cost(self):
        '''one agents, one optional task and a required one. Checking if the option mixed_cost works'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=1, M=2)
        MyTasks.OptionalTasks['Task 0'] = False
        MyTasks.OptionalTasks['Task 1'] = True
        MyTasks.TaskReward['Task 0'] = 0.
        MyTasks.TaskReward['Task 1'] = 1.
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .5
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = .5
        MyTasks.DependencyList['Task 0'] = ['Task 1']
        MyTasks.MaxLatency['Task 0']['Task 1'] = 999.
        MyTasks.validate()
        MyOptions = {}
        MyOptions["CostFunction"] = {"energy": 1.0,
                                     "total_task_reward": 1.0, "total_time": 0.0}

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 0')

    def test_energy_cost(self):
        '''two agents, three tasks. Checking if the energy option works'''
        _, _, MyAgentCap, MyTasks, MyCommNet, MyOptions = self.initializeVars(
            N=2, M=3)
        MyAgentCap.EnergyCost['Agent 0']['Task 0'] = 1
        MyAgentCap.EnergyCost['Agent 0']['Task 1'] = 3.
        MyAgentCap.EnergyCost['Agent 0']['Task 2'] = 1
        MyAgentCap.EnergyCost['Agent 1']['Task 0'] = 3.
        MyAgentCap.EnergyCost['Agent 1']['Task 1'] = 1
        MyAgentCap.EnergyCost['Agent 1']['Task 2'] = 3.
        MyAgentCap.ComputationLoad['Agent 0']['Task 0'] = .2
        MyAgentCap.ComputationLoad['Agent 0']['Task 1'] = .2
        MyAgentCap.ComputationLoad['Agent 0']['Task 2'] = .2
        MyAgentCap.ComputationLoad['Agent 1']['Task 0'] = .2
        MyAgentCap.ComputationLoad['Agent 1']['Task 1'] = .2
        MyAgentCap.ComputationLoad['Agent 1']['Task 2'] = .2

        MyTasks.DependencyList['Task 1'] = ['Task 0']
        MyTasks.DependencyList['Task 2'] = ['Task 1']
        MyTasks.MaxLatency['Task 1']['Task 0'] = 999.
        MyTasks.MaxLatency['Task 2']['Task 1'] = 999.

        MyTasks.validate()
        MyOptions = {}
        MyOptions["CostFunction"] = {"energy": 1.0,
                                     "total_task_reward": 0., "total_time": 0.0}

        for _solver in self.available_solvers():
            Solver = self.call_solver(
                MyAgentCap, MyTasks, MyCommNet, MyOptions, solver=_solver)
            (TaskAssignment, _) = Solver.schedule()
            self.assertTrue(TaskAssignment['TaskAgent']['Task 0'] == 'Agent 0')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 1'] == 'Agent 1')
            self.assertTrue(TaskAssignment['TaskAgent']['Task 2'] == 'Agent 0')


if __name__ == '__main__':
    unittest.main()
