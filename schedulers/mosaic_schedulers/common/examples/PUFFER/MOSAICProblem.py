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

import mosaic_schedulers.schedulers.tv_milp as MOSAICSolver
from mosaic_schedulers.common.utilities import NetworkPartitioning
from mosaic_schedulers.common.plotting import SchedulePlotter

import networkx as nx
import numpy as np
import colorsys
import time
import json
try:
    import cplex
    CPLEX_Available = True
except:
    CPLEX_Available = False


class MOSAICProblem:
    """ An instance of the MOSAIC scheduler.
    Exposes the following methods:
    - __init__: build the problem
    - partition: partitions the network in sub-networks.
    - schedule: calls the MILP scheduler.
    - partition: partitions the network
    - buildScheduler: builds the scheduler
    - getOptimizationTerminator: returns an object that can stop the scheduler in anytime fashion
    - schedule: runs the optimizer
    - getTaskColors: returns task colors for plotting
    - _createFeasibleSolution: returns a feasible solution
    """

    def __init__(self,
                 problemState,
                 maxLatency=1e-3,
                 minBandwidth=0.1,
                 maxHops=float("inf"),
                 TimeStep=2,
                 solverClockLimit=1e75,
                 solverDetTicksLimit=1e75,
                 solverParameters={},
                 scaleTaskTime=1.):
        '''Inputs:
        - problemState, a description of the problem state in JSON format. See the
            usage example in __main__.in
        - maxLatency: the maximum latency between nodes, used to cluster the  network
            in multiple groups if mode 'Latency' is used. Default = 1e-3
        - minBandwidth, also used to cluster the network in groups if mode
            'Bandwidth' or 'BandwidthHops' is used. Default = 0.1
        - maxHops,  used to cluster the network in groups if mode
            'Hops' or 'BandwidthHops' is used. Default = float("inf")
        - TimeStep: the time step of the MILP solver
        - solverClockLimit (default=1e75), the maximum time allotted to the solver
            (in seconds). Note that this is NOT a deterministic time limit.
        - solverDetTicksLimit (default=1e75), the maximum number of ticks allotted
            to the solver. Tics are an internal deterministic measure of solver
            progress: the mapping from ticks to time is machine- and
            load-specific.  This limit should be used to ensure that multiple
            instances of the problem terminate simultaneously.
        - solverParameters, a dictionary of parameters for the MIP solver.
            Default: empty.
        - scaleTaskTime, a parameter to scale all the task processing times.

        '''
        self.problemState = problemState
        self.maxLatency = maxLatency
        self.minBandwidth = minBandwidth
        self.maxHops = maxHops
        self.TimeStep = TimeStep
        self.solverClockLimit = solverClockLimit
        self.solverDetTicksLimit = solverDetTicksLimit
        self.solverParameters = solverParameters

        self.BaseStationName = 'base_station'

        self.isPartitioned = False
        self.isSetUp = False
        self.isScheduled = False
        self.scaleTaskTime = scaleTaskTime

    def partition(self, mode='Bandwidth'):
        '''
        Partitions the graph according to the latency or bandwidth between PUFFERs.
        Input: mode, the mode used to partition the network.
        Mode can be:
        - Bandwidth: removes all edges with bandwidth lower than
            self.minBandwidth and returns the resulting connected components.
        - Latency: estimates the latency of every edge according to inter-agent
            distance and light-speed delays, and returns clusters where all
            agents are within self.maxLatency of each other.
        - Hops: returns clusters where all agents are within self.maxHops of
            each other.
        '''
        # Build a NetworkX graph
        AgentsDict = self.problemState['nodes'].copy()
        # if self.BaseStationName not in AgentsDict.keys():
        #     AgentsDict[self.BaseStationName] = {'position': [0.0, 0.0, 0.0]}

        Agents = AgentsDict.items()

        G = nx.DiGraph()
        G.add_nodes_from(Agents)
        if mode == 'Latency':
            for node in G.node():
                G.node[node]['position_2d'] = G.node[node]['position'][:-1]

        Links_list = []
        if mode == 'Latency':
            for key, val in self.problemState['link_state'].items():
                node0 = key[0]
                node1 = key[1]
                _val = val.copy()
                _val['length'] = np.linalg.norm(np.array(
                    AgentsDict[node0]['position']) - np.array(AgentsDict[node1]['position']))
                _val['LoS latency'] = _val['length'] / 299792458.
                Links_list.append((node0, node1, _val))
            G.add_edges_from(Links_list)
            # Partition the network
            self.ConnectedComponents = NetworkPartitioning.partition_by_latency(
                G, max_diameter=self.maxLatency, distance_label='LoS latency')
        elif mode == 'Bandwidth':
            for key, val in self.problemState['link_state'].items():
                node0 = key[0]
                node1 = key[1]
                if val['bandwidth'] >= self.minBandwidth:
                    Links_list.append((node0, node1, val))
            G.add_edges_from(Links_list)
            # Partition the network
            self.ConnectedComponents = NetworkPartitioning.partition_by_latency(
                G, max_diameter=len(G.nodes()))
        elif mode == 'Hops':
            for key, val in self.problemState['link_state'].items():
                node0 = key[0]
                node1 = key[1]
                _val = val.copy()
                _val['length'] = 1
                Links_list.append((node0, node1, _val))
            G.add_edges_from(Links_list)
            # Partition the network
            self.ConnectedComponents = NetworkPartitioning.partition_by_latency(
                G, max_diameter=self.maxHops, distance_label='length')
        elif mode == 'BandwidthHops':
            for key, val in self.problemState['link_state'].items():
                node0 = key[0]
                node1 = key[1]
                if val['bandwidth'] >= self.minBandwidth:
                    Links_list.append((node0, node1, val))
            G.add_edges_from(Links_list)
            # Partition the network
            self.ConnectedComponents = NetworkPartitioning.partition_by_latency(
                G, max_diameter=self.maxHops)
        else:
            raise NotImplementedError

        NetworkPartitioning.label_connected_components(
            G, self.ConnectedComponents, plot=False, pos=G.nodes('position_2d'))
        self.G = G

        self.isPartitioned = True

        return self.ConnectedComponents

    def buildScheduler(self):
        ''' Creates the problem instance to be solved by the scheduler and calls it '''
        if self.isPartitioned is False:
            self.partition()

        problemState = self.problemState
        BaseStationName = self.BaseStationName
        # Build a sub-problem just for the agent's partition

        # Find where the agent is
        agent = problemState['agent']
        AgentNames = [agent]
        for component in self.ConnectedComponents:
            if agent in component:
                AgentNames = component
                break
        print("Agent names in partition:", AgentNames,
              "All partitions:", self.ConnectedComponents)

        PufferNames = list(AgentNames)
        if BaseStationName in PufferNames:
            PufferNames.remove(BaseStationName)

        # Build the bandwidth graph for the relevant connected component
        Thor_s = problemState['time_horizon']
        TimeStep = self.TimeStep

        Thor = int(float(Thor_s) / float(TimeStep))
        # A task that can't be done by an agent is set to last this long on the agent
        InfeasibleTime = 2 * Thor_s

        CommWindowsBandwidth = {}
        for t in range(Thor):
            CommWindowsBandwidth[t] = {}
            for Agent1 in AgentNames:
                CommWindowsBandwidth[t][Agent1] = {}
                for Agent2 in AgentNames:
                    if (Agent1, Agent2) in problemState['link_state'].keys():
                        CommWindowsBandwidth[t][Agent1][Agent2] = problemState['link_state'][(
                            Agent1, Agent2)]['bandwidth']
                    else:
                        CommWindowsBandwidth[t][Agent1][Agent2] = 0

        # Extract the science availability
        # TODO How many samples can you take within the given horizon
        achievable_science_in_horizon = 3

        pufferScienceAvailability = {}
        for puffer in PufferNames:
            in_science_zone = problemState['nodes'][puffer]['in_science_zone']
            pufferScienceAvailability[puffer] = achievable_science_in_horizon * \
                in_science_zone

        # Here we specify the tasks that _each PUFFER_ should perform. We will assemble them later.

        # Reward for performing a task
        PufferTaskReward_housekeeping = {
            '2. Short Range Image': 0,
            '3. VO Localization': 0,
            '4. Plan Path (VO)': 0,
            '5. Send Drive Command': 0,
        }

        PufferTaskReward_science = {
            '6. Take Sample': 5,
            '7. Analyse Sample': 10,
            '8. Store Sample': 20,
        }

        # pufferScienceReward = None  # This would allow us to override the task reward on a per-target basis

        # Which tasks are optional for performing a task

        PufferOptionalTask_housekeeping = {
            '2. Short Range Image': False,
            '3. VO Localization': False,
            '4. Plan Path (VO)': False,
            '5. Send Drive Command': False,
        }

        PufferOptionalTask_science = {
            '6. Take Sample': True,
            '7. Analyse Sample': True,
            '8. Store Sample': True,
        }

        # Size of the data products
        PufferProductSizes_housekeeping = {
            '2. Short Range Image': 8.,
            '3. VO Localization': .1,
            '4. Plan Path (VO)': .1,
            '5. Send Drive Command': .1,
        }

        PufferProductSizes_science = {
            '6. Take Sample': 15,
            '7. Analyse Sample': 1,
            '8. Store Sample': .1,
        }

        # Computation times for each task (in seconds)
        # The base station's computation time
        scaleTaskTime = float(self.scaleTaskTime)
        BaseComputationTimeP_housekeeping = {
            '2. Short Range Image': InfeasibleTime,
            '3. VO Localization': 1 * scaleTaskTime,
            '4. Plan Path (VO)': .1 * scaleTaskTime,
            '5. Send Drive Command': InfeasibleTime
        }

        BaseComputationTimeP_science = {
            '6. Take Sample': InfeasibleTime,
            '7. Analyse Sample': 1 * scaleTaskTime,
            '8. Store Sample': .1 * scaleTaskTime
        }

        # The computation time for a PUFFER doing its own tasks
        PufferComputationTimeP_self_housekeeping = {
            '2. Short Range Image': 3. * scaleTaskTime,
            '3. VO Localization': 10. * scaleTaskTime,
            '4. Plan Path (VO)': 10. * scaleTaskTime,
            '5. Send Drive Command': .1 * scaleTaskTime
        }

        PufferComputationTimeP_self_science = {
            '6. Take Sample': 5. * scaleTaskTime,
            '7. Analyse Sample': 10. * scaleTaskTime,
            '8. Store Sample': InfeasibleTime
        }

        # The computation time for a PUFFER doing another PUFFER's tasks (e.g., it can't drive on behalf of someone else)
        PufferComputationTimeP_other_housekeeping = {
            '2. Short Range Image': InfeasibleTime,
            '3. VO Localization': 10. * scaleTaskTime,
            '4. Plan Path (VO)': 10. * scaleTaskTime,
            '5. Send Drive Command': InfeasibleTime
        }

        PufferComputationTimeP_other_science = {
            '6. Take Sample': InfeasibleTime,
            '7. Analyse Sample': 10. * scaleTaskTime,
            '8. Store Sample': InfeasibleTime
        }

        # Map from seconds to time steps
        BaseComputationTimeP_housekeeping = {k: int(np.ceil(float(
            v) / float(TimeStep))) for k, v in BaseComputationTimeP_housekeeping.items()}
        BaseComputationTimeP_science = {k: int(np.ceil(float(
            v) / float(TimeStep))) for k, v in BaseComputationTimeP_science.items()}
        PufferComputationTimeP_self_housekeeping = {k: int(np.ceil(float(
            v) / float(TimeStep))) for k, v in PufferComputationTimeP_self_housekeeping.items()}
        PufferComputationTimeP_self_science = {k: int(np.ceil(float(
            v) / float(TimeStep))) for k, v in PufferComputationTimeP_self_science.items()}
        PufferComputationTimeP_other_housekeeping = {k: int(np.ceil(float(
            v) / float(TimeStep))) for k, v in PufferComputationTimeP_other_housekeeping.items()}
        PufferComputationTimeP_other_science = {k: int(np.ceil(float(
            v) / float(TimeStep))) for k, v in PufferComputationTimeP_other_science.items()}

        # Dependency map for the PUFFERs. Recall that dependencies are in CNF.
        PufferDependencyList_housekeeping = {
            '2. Short Range Image': [[]],
            '3. VO Localization': [['2. Short Range Image']],
            '4. Plan Path (VO)': [['3. VO Localization']],
            '5. Send Drive Command': [['4. Plan Path (VO)']]
        }

        PufferDependencyList_science = {
            '6. Take Sample': [[]],
            '7. Analyse Sample': [['6. Take Sample']],
            '8. Store Sample': [['7. Analyse Sample']]
        }

        # Incompatible tasks
        PufferIncompatibleTasks_housekeeping = []
        PufferIncompatibleTasks_science = []

        # Here we build the task list for the entire system by merging the PUFFER tasks
        AllOptionalTasks = {}
        AllTaskReward = {}
        AllProductSizes = {}
        AllDependencyList = {}
        AllIncompatibleTasks = []

        # Optional tasks, rewards, product sizes, and dependencies
        for pufferName in PufferNames:
            # Add tasks
            Temp_PufferOptionalTask = {'{}:'.format(
                pufferName) + k: v for k, v in PufferOptionalTask_housekeeping.items()}
            Temp_PufferTaskReward = {'{}:'.format(
                pufferName) + k: v for k, v in PufferTaskReward_housekeeping.items()}
            Temp_PufferProductSizes = {'{}:'.format(
                pufferName) + k: v for k, v in PufferProductSizes_housekeeping.items()}

            for scienceNo in range(pufferScienceAvailability[pufferName]):
                Temp_PufferOptionalTask.update({'{}:{}'.format(
                    pufferName, scienceNo) + k: v for k, v in PufferOptionalTask_science.items()})

                # if pufferScienceReward is None:
                Temp_PufferTaskReward.update({'{}:{}'.format(
                    pufferName, scienceNo) + k: v for k, v in PufferTaskReward_science.items()})
                # else:
                #    assert pufferScienceReward[pufferName][scienceNo].keys() == PufferTaskReward_science.keys(), "ERROR: PUFFER science reward has inconsistent keys"
                #    Temp_PufferTaskReward.update({'{}:{}'.format(pufferName, scienceNo) + k: v for k, v in pufferScienceReward[pufferName][scienceNo].items()})

                Temp_PufferProductSizes.update({'{}:{}'.format(
                    pufferName, scienceNo) + k: v for k, v in PufferProductSizes_science.items()})

            for k, v in PufferDependencyList_housekeeping.items():
                AllDeps = []
                for ConjDependency in v:
                    AllDeps.append(
                        ['{}:'.format(pufferName) + DisjDependency for DisjDependency in ConjDependency])
                AllDependencyList.update(
                    {'{}:'.format(pufferName) + k: AllDeps})

            for scienceNo in range(pufferScienceAvailability[pufferName]):
                for k, v in PufferDependencyList_science.items():
                    AllDeps = []
                    for ConjDependency in v:
                        AllDeps.append(['{}:{}'.format(
                            pufferName, scienceNo) + DisjDependency for DisjDependency in ConjDependency])
                    AllDependencyList.update(
                        {'{}:{}'.format(pufferName, scienceNo) + k: AllDeps})

            for BadPairing in PufferIncompatibleTasks_housekeeping:
                Temp_BadPairing = ['{}:'.format(
                    pufferName) + Task for Task in BadPairing]
                AllIncompatibleTasks.append(Temp_BadPairing)
            for scienceNo in range(pufferScienceAvailability[pufferName]):
                for BadPairing in PufferIncompatibleTasks_science:
                    Temp_BadPairing = ['{}:{}'.format(
                        pufferName, scienceNo) + Task for Task in BadPairing]
                    AllIncompatibleTasks.append(Temp_BadPairing)
            AllOptionalTasks.update(Temp_PufferOptionalTask)
            AllTaskReward.update(Temp_PufferTaskReward)
            AllProductSizes.update(Temp_PufferProductSizes)

        # Computation time, load, and initial information
        AllComputationTime = {}
        AllComputationLoad = {}
        AllInitialInformation = {}

        for pufferName in PufferNames:
            for task, tasktime in PufferComputationTimeP_self_housekeeping.items():
                AllComputationTime.update(
                    {'{}:'.format(pufferName) + task: {pufferName: tasktime}})
                AllComputationLoad.update(
                    {'{}:'.format(pufferName) + task: {pufferName: 1.}})
                AllInitialInformation.update(
                    {'{}:'.format(pufferName) + task: {pufferName: False}})
            if BaseStationName in AgentNames:  # If the base station is in the group
                for task, tasktime in BaseComputationTimeP_housekeeping.items():
                    AllComputationTime['{}:'.format(
                        pufferName) + task][BaseStationName] = tasktime
                    AllComputationLoad['{}:'.format(pufferName) +
                                       task][BaseStationName] = 1.
                    AllInitialInformation['{}:'.format(
                        pufferName) + task][BaseStationName] = False

            for otherPuffer in PufferNames:
                if otherPuffer != pufferName:
                    for task, tasktime in PufferComputationTimeP_other_housekeeping.items():
                        AllComputationTime['{}:'.format(
                            pufferName) + task][otherPuffer] = tasktime
                        AllComputationLoad['{}:'.format(pufferName) +
                                           task][otherPuffer] = 1.
                        AllInitialInformation['{}:'.format(
                            pufferName) + task][otherPuffer] = False

            for scienceNo in range(pufferScienceAvailability[pufferName]):
                for task, tasktime in PufferComputationTimeP_self_science.items():
                    AllComputationTime.update(
                        {'{}:{}'.format(pufferName, scienceNo) + task: {pufferName: tasktime}})
                    AllComputationLoad.update(
                        {'{}:{}'.format(pufferName, scienceNo) + task: {pufferName: 1.}})
                    AllInitialInformation.update(
                        {'{}:{}'.format(pufferName, scienceNo) + task: {pufferName: False}})
                if BaseStationName in AgentNames:  # If the base station is in the group
                    for task, tasktime in BaseComputationTimeP_science.items():
                        AllComputationTime['{}:{}'.format(
                            pufferName, scienceNo) + task][BaseStationName] = tasktime
                        AllComputationLoad['{}:{}'.format(
                            pufferName, scienceNo) + task][BaseStationName] = 1.
                        AllInitialInformation['{}:{}'.format(
                            pufferName, scienceNo) + task][BaseStationName] = False

                for otherPuffer in PufferNames:
                    if otherPuffer != pufferName:
                        for task, tasktime in PufferComputationTimeP_other_science.items():
                            AllComputationTime['{}:{}'.format(
                                pufferName, scienceNo) + task][otherPuffer] = tasktime
                            AllComputationLoad['{}:{}'.format(
                                pufferName, scienceNo) + task][otherPuffer] = 1.
                            AllInitialInformation['{}:{}'.format(
                                pufferName, scienceNo) + task][otherPuffer] = False

        # We define task colors for plotting. This is not needed to get the schedule, just to plot it.
        AllTaskColors = {}

        # Optional tasks, rewards, product sizes, and dependencies
        PufferTaskList_housekeeping = PufferOptionalTask_housekeeping.keys()
        PufferTaskList_science = PufferOptionalTask_science.keys()

        PufferNo = 0
        N_puffers = len(PufferNames)

        for pufferName in PufferNames:
            TempPufferHSVColor = np.array(
                [float(PufferNo) / float(N_puffers), 0., .85])
            PufferTaskCounter = 0

            NumPufferTasks = len(
                list(PufferTaskList_housekeeping) + list(PufferTaskList_science))
            for PufferTask in PufferTaskList_housekeeping:
                TempTaskHSVColor = np.array(
                    [0., .2 + .8 * (1. - float(PufferTaskCounter) / float(NumPufferTasks - 1)), 0.])
                AllTaskColors['{}:'.format(
                    pufferName) + PufferTask] = np.array(colorsys.hsv_to_rgb(TempPufferHSVColor[0] + TempTaskHSVColor[0], TempPufferHSVColor[1] + TempTaskHSVColor[1], TempPufferHSVColor[2] + TempTaskHSVColor[2]))
                PufferTaskCounter += 1
            for PufferTask in PufferTaskList_science:
                TempTaskHSVColor = np.array(
                    [0., .2 + .8 * (1. - float(PufferTaskCounter) / float(NumPufferTasks - 1)), 0.])
                for scienceNo in range(pufferScienceAvailability[pufferName]):
                    AllTaskColors['{}:{}'.format(
                        pufferName, scienceNo) + PufferTask] = np.array(colorsys.hsv_to_rgb(TempPufferHSVColor[0] + TempTaskHSVColor[0], TempPufferHSVColor[1] + TempTaskHSVColor[1], TempPufferHSVColor[2] + TempTaskHSVColor[2]))
                PufferTaskCounter += 1

            PufferNo += 1

        self.AllTaskColors = AllTaskColors

        # Create a feasible solution
        HousekeepingTasks = list(
            PufferComputationTimeP_self_housekeeping.keys())
        HousekeepingTasks.sort()

        MIP_vars = []
        MIP_vals = []
        for pufferName in PufferNames:
            actionTime = 0
            for task in HousekeepingTasks:
                MIP_vars += ["X[{0},{0}:{1},{2}]".format(
                    pufferName, task, actionTime)]
                MIP_vals += [1]
                actionTime += PufferComputationTimeP_self_housekeeping[task]
                for knowledgeTime in range(actionTime, Thor):
                    MIP_vars += ["D[{0},{0}:{1},{2}]".format(
                        pufferName, task, knowledgeTime)]
                    MIP_vals += [1]
        self.FeasibleSolution = [MIP_vars, MIP_vals]

        # Assemble inputs for the solver constructor
        Tasks = MOSAICSolver.MILPTasks(
            OptionalTasks=AllOptionalTasks,
            TaskReward=AllTaskReward,
            ProductsSize=AllProductSizes,
            DependencyList=AllDependencyList,
            IncompatibleTasks=AllIncompatibleTasks,
        )

        AgentCapabilities = MOSAICSolver.MILPAgentCapabilities(
            ComputationTime=AllComputationTime,
            ComputationLoad=AllComputationLoad,
            InitialInformation=AllInitialInformation
        )

        CommunicationNetwork = MOSAICSolver.CommunicationNetwork(
            CommWindowsBandwidth=CommWindowsBandwidth
        )

        self.Tasks = Tasks
        self.AgentCapabilities = AgentCapabilities
        self.Thor = Thor

        # Create a feasible solution
        # MIP_vars = []
        # MIP_vals = []
        # for pufferName in PufferNames:
        #     actionTime = 0
        #     for task in HousekeepingTasks:
        #         MIP_vars += ["X[{0},{0}:{1},{2}]".format(pufferName, task, actionTime)]
        #         MIP_vals += [1]
        #         actionTime += PufferComputationTimeP_self_housekeeping[task]
        #         for knowledgeTime in range(actionTime, Thor):
        #             MIP_vars += ["D[{0},{0}:{1},{2}]".format(pufferName, task, knowledgeTime)]
        #             MIP_vals += [1]
        # self.FeasibleSolution = [MIP_vars, MIP_vals]
        if CPLEX_Available:
            FeasibleStartDict = self._createFeasibleSolution(
                Tasks, AgentCapabilities, CommunicationNetwork.CommWindowsBandwidth, PufferNames, PufferComputationTimeP_self_housekeeping, Thor)
            MIP_vars = []
            MIP_vals = []
            for key in FeasibleStartDict:
                MIP_vars.append(key)
                MIP_vals.append(FeasibleStartDict[key])
            self.FeasibleSolution = [MIP_vars, MIP_vals]

        Options = {'Packet Comms': False}  # Scheduler options/params

        if CPLEX_Available:
            self.scheduler = MOSAICSolver.MOSAICCPLEXScheduler(
                Thor, AgentCapabilities, Tasks, CommunicationNetwork, self.TimeStep, Options, Verbose=False)
            self.scheduler.setTimeLimits(
                DetTicksLimit=self.solverDetTicksLimit, ClockTimeLimit=self.solverClockLimit)
            self.schedulerTerminator = self.scheduler.getOptimizationTerminator()
        else:
            self.scheduler = MOSAICSolver.MOSAICGLPKScheduler(
                Thor, AgentCapabilities, Tasks, CommunicationNetwork, self.TimeStep, Options, Verbose=False)
        # timeLimit = {'Seconds': self.solverTimeLimit, 'Ticks per second': self.ticksPerSecond}

        self.scheduler.setSolverParameters(self.solverParameters)
        self.isSetUp = True

    def _createFeasibleSolution(self, Tasks, AgentCapabilities, CommWindowsBandwidth, PufferNames, PufferComputationTimeP_self_housekeeping, Thor):
        TasksList = list(Tasks.OptionalTasks.keys())
        AgentsList = list(
            AgentCapabilities.ComputationLoad[TasksList[0]].keys())
        HousekeepingTasks = list(
            PufferComputationTimeP_self_housekeeping.keys())
        HousekeepingTasks.sort()
        FeasibleStart = {}
        X_name = []
        for i in AgentsList:
            for m in TasksList:
                for t in range(Thor - AgentCapabilities.ComputationTime[m][i]):
                    if t >= 0:
                        X_name.append('X[{},{},{}]'.format(i, m, t))
                        FeasibleStart['X[{},{},{}]'.format(i, m, t)] = 0.
        D_name = []
        for i in AgentsList:
            for m in TasksList:
                for t in range(Thor):
                    D_name.append('D[{},{},{}]'.format(i, m, t))
                    FeasibleStart['D[{},{},{}]'.format(i, m, t)] = 0.
        C_name = []
        for i in AgentsList:
            for j in AgentsList:
                for m in TasksList:
                    for t in range(Thor):
                        if CommWindowsBandwidth[t][i][j] > 0:
                            C_name.append('C[{},{},{},{}]'.format(i, j, m, t))
                            FeasibleStart['C[{},{},{},{}]'.format(
                                i, j, m, t)] = 0.

        for pufferName in PufferNames:
            actionTime = 0
            for task in HousekeepingTasks:
                task_key = "X[{0},{0}:{1},{2}]".format(
                    pufferName, task, actionTime)
                assert task_key in FeasibleStart.keys()
                FeasibleStart[task_key] = 1.
                actionTime += PufferComputationTimeP_self_housekeeping[task]
                for knowledgeTime in range(actionTime, Thor):
                    know_key = "D[{0},{0}:{1},{2}]".format(
                        pufferName, task, knowledgeTime)
                    assert know_key in FeasibleStart.keys()
                    FeasibleStart[know_key] = 1.

        return FeasibleStart

    def getOptimizationTerminator(self):
        if self.isSetUp is not True:
            raise Exception(
                "Problem is not set up yet, terminator not available")
        return self.schedulerTerminator

    def schedule(self):
        setup_start_time = time.process_time()
        if self.isSetUp is False:
            self.buildScheduler()
        # Set feasible solution
        self.scheduler.setUp()
        if CPLEX_Available:
            try:
                self.scheduler.setMIPStart(
                    self.FeasibleSolution, self.scheduler.cpx.MIP_starts.effort_level.check_feasibility)
            except cplex.exceptions.CplexSolverError:
                print(
                    "WARNING: default feasible solution is problematic. This should never happen")
                pass

        setup_end_time = time.process_time() - setup_start_time
        remaining_solver_time = self.solverClockLimit - setup_end_time
        if remaining_solver_time < 0:
            print("ERROR: Not enough time to solve - timeout while building the problem. Consider increasing ClockTimeLimit")
            return
        self.scheduler.setTimeLimits(
            ClockTimeLimit=remaining_solver_time, DetTicksLimit=self.solverDetTicksLimit)
        self.scheduler.schedule()
        self.Schedule = self.scheduler.formatOutput(version=2)
        self.isScheduled = True
        return self.Schedule

    def getTaskColors(self):
        if self.isSetUp is False:
            raise Exception(
                "Please set up the problem [problem.setUp()] before calling getTaskColors!")
        return self.AllTaskColors


def stringifyState(problemState):
    stringState = problemState.copy()
    newLinkState = {}
    for key, val in problemState['link_state'].items():
        # Translate to string. Better hope names do not contain semicolons...
        newKey = key[0] + ':' + key[1]
        newLinkState[newKey] = val
    newContactPlan = {}
    for key, val in problemState['contact_plan'].items():
        # Translate to string. Better hope names do not contain semicolons...
        newKey = key[0] + ':' + key[1]
        newContactPlan[newKey] = val
    stringState['link_state'] = newLinkState
    stringState['contact_plan'] = newContactPlan
    return stringState


def destringifyState(stringState):
    problemState = stringState.copy()
    newLinkState = {}
    for key, val in stringState['link_state'].items():
        linkEnds = key.split(':')
        assert len(
            linkEnds) == 2, 'ERROR: could not convert keys in problemState to strings. Do the agent names contain colons?'
        linkHead = linkEnds[0]
        linkTail = linkEnds[1]
        linkID = (linkHead, linkTail)
        newLinkState[linkID] = val
    newContactPlan = {}
    for key, val in stringState['contact_plan'].items():
        linkEnds = key.split(':')
        assert len(
            linkEnds) == 2, 'ERROR: could not convert keys in problemState to strings. Do the agent names contain colons?'
        linkHead = linkEnds[0]
        linkTail = linkEnds[1]
        linkID = (linkHead, linkTail)
        newContactPlan[linkID] = val

    problemState['link_state'] = newLinkState
    problemState['contact_plan'] = newContactPlan
    return problemState


def stringifyTupleDict(tuple_dict):
    json_dict = {}
    for key, val in tuple_dict.items():
        str_key = ":".join([entry for entry in key])
        json_dict[str_key] = val
    return json_dict


def destringifyJSONDict(json_dict):
    tuple_dict = {}
    for key, val in json_dict.items():
        tup_key = tuple(key.split(':'))
        tuple_dict[tup_key] = val
    return tuple_dict


if __name__ == "__main__":
    # Scenario 4
    problemState = {
        'time': 1002.0,
        'agent': 'puffer1',
        'time_horizon': 60,
        'link_state': {
            ('puffer2', 'puffer1'): {'bandwidth': 11.0, 'time': 832.9},
            ('puffer1', 'puffer3'): {'bandwidth': 0.0, 'time': 831.2},
            ('puffer2', 'puffer3'): {'bandwidth': 11.0, 'time': 832.9},
            ('puffer1', 'base_station'): {'bandwidth': 0.0, 'time': 827.5},
            ('puffer3', 'base_station'): {'bandwidth': 11.0, 'time': 831.2},
            ('puffer3', 'puffer1'): {'bandwidth': 0.0, 'time': 831.2},
            ('puffer3', 'puffer2'): {'bandwidth': 11.0, 'time': 832.9},
            ('puffer2', 'base_station'): {'bandwidth': 0.0, 'time': 832.9},
            ('puffer1', 'puffer2'): {'bandwidth': 11.0, 'time': 832.9},
            ('base_station', 'puffer1'): {'bandwidth': 0.0, 'time': 831.2},
            ('base_station', 'puffer2'): {'bandwidth': 0.0, 'time': 831.2},
            ('base_station', 'puffer3'): {'bandwidth': 11.0, 'time': 831.2},
        },
        'nodes': {
            'puffer3': {'in_science_zone': False},
            'puffer2': {'in_science_zone': False},
            'puffer1': {'in_science_zone': True},
        },
        'contact_plan': {
            ('puffer1', 'puffer3'): [{'bandwidth': 0.0, 'time_start': 831.2, 'time_end': 1.7976931348623157e+308}],
            ('puffer1', 'base_station'): [{'bandwidth': 11.0, 'time_start': 827.5, 'time_end': 1.7976931348623157e+308}],
            ('puffer1', 'puffer2'): [{'bandwidth': 0.0, 'time_start': 832.9, 'time_end': 1.7976931348623157e+308}],
            ('puffer2', 'base_station'): [{'bandwidth': 0.0, 'time_start': 827.5, 'time_end': 1.7976931348623157e+308}],
            ('puffer3', 'base_station'): [{'bandwidth': 0.0, 'time_start': 831.2, 'time_end': 1.7976931348623157e+308}],
            ('puffer3', 'puffer1'): [{'bandwidth': 0.0, 'time_start': 831.2, 'time_end': 1.7976931348623157e+308}],
            ('puffer3', 'puffer2'): [{'bandwidth': 11.0, 'time_start': 832.9, 'time_end': 1.7976931348623157e+308}],
        }
    }

    solverParameters = {('read', 'datacheck'): 1,
                        ('mip', 'strategy', 'variableselect'): 4,
                        ('mip', 'strategy', 'heuristicfreq'): 50,
                        ('mip', 'limits', 'cutpasses'): 1}

    # Create a problem instance
    MProblem = MOSAICProblem(problemState,
                             # Disconnect agents if latency is higher than this value and partition mode is 'Latency'
                             maxLatency=1e-7,
                             minBandwidth=6,    # Disconnect agents if bandwidth is lower than this value and partition mode is 'Bandwidth'
                             # Emergency stop: stop optimization if it takes longer than this many seconds (non-deterministic)
                             solverClockLimit=60,
                             # Deterministic time limit: Stop optimization if it takes longer than this many ticks
                             solverDetTicksLimit=10000,
                             solverParameters=solverParameters,  # Parameters for the MIP solver
                             )

    # Partition the network
    ConnectedComponents = MProblem.partition(
        mode='Bandwidth')  # Partition the network
    # Build the scheduler
    MProblem.buildScheduler()
    if CPLEX_Available:
        # Get an object to terminate the optimization asynchronously
        OptimizationTerminator = MProblem.getOptimizationTerminator()
    # Call the scheduler, get back a JSON solution
    Schedule = MProblem.schedule()

    '''
    Get the problem state (to know why it stopped). A list of stopping criteria
    and corresponding exit codes can be found at:
    https://www.ibm.com/support/knowledgecenter/SSSA5P_12.8.0/ilog.odms.cplex.help/refcallablelibrary/macros/homepagesolutionstatusMIP.html
    Relevant stopping criteria:
    Deterministic time limit, feasible (synchronization guaranteed): "CPXMIP_DETTIME_LIM_FEAS", 131
    Deterministic time limit, INfeasible : "CPXMIP_DETTIME_LIM_INFEAS", 132
    Nondeterministic time limit, feasible (synchronization not guaranteed): "CPXMIP_TIME_LIM_FEAS", 107
    Nondeterministic time limit, INfeasible: "CPXMIP_TIME_LIM_INFEAS", 108
    Aborted by user "CPX_STAT_ABORT_USER", 13
    '''
    solverState = MProblem.scheduler.getSolverState()

    if CPLEX_Available:
        # Get the solution in a format that can be used to seed future MILPs
        MIPStart = MProblem.scheduler.getMIPStart()
        MProblem.scheduler.setMIPStart(MIPStart)

    # Get task colors (for plotting)
    TaskColors = MProblem.getTaskColors()
    # Plot the schedule
    SchedulePlotter(Schedule, TaskColors, show=True, save=False)
