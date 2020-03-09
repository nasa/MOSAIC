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
import networkx as nx
import numpy as np
import colorsys
import time
import json
import sys
try:
    import cplex
    CPLEX_Available = True
except:
    CPLEX_Available = False


class MOSAICProblem:
    """ An instance of the MOSAIC scheduler.
    Exposes the following methods:
    - __init__
    - createJSON: creates the JSON problem description that the schedulers can digest
    - partition: partitions the network in sub-networks.
    - buildScheduler: builds the scheduler
    - getTaskColors: returns task colors for plotting
    """

    def __init__(self,
                 ScenarioParameters,
                 minBandwidth=0.01,
                 maxHops=sys.maxsize,
                 TimeStep=2,
                 solverClockLimit=1e75,
                 solverDetTicksLimit=1e75,
                 solverParameters={}):
        '''Inputs:
        - ScenarioParameters, a description of the PUFFER scenario parameters in JSON format.
        See sample_input and ScenarioGenerator.py for examples.
        - minBandwidth, also used to cluster the network in groups if mode
            'Bandwidth' or 'BandwidthHops' is used. Default = 0.01
        - maxHops,  used to cluster the network in groups if mode
            'Hops' or 'BandwidthHops' is used. Default = sys.maxint
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

        '''
        self.ScenarioParameters = json.loads(ScenarioParameters)
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

    def partition(self, mode='Bandwidth'):
        '''
        Partitions the graph according to the latency or bandwidth between PUFFERs.
        Input: mode, the mode used to partition the network.
        Mode can be:
        - Bandwidth: removes all edges with bandwidth lower than
            self.minBandwidth and returns the resulting connected components.
        - Hops: returns clusters where all agents are within self.maxHops of
            each other.
        - BandwidthHops: filter by bandwidth and then cluster by hops
        '''
        # Build a NetworkX graph
        Agents = self.ScenarioParameters['Agents']
        # if self.BaseStationName not in AgentsDict.keys():
        #     AgentsDict[self.BaseStationName] = {'position': [0.0, 0.0, 0.0]}

        G = nx.DiGraph()
        G.add_nodes_from(Agents)

        Links_list = []
        if mode == 'Bandwidth':
            for link in self.ScenarioParameters['CommunicationNetwork']:
                # for key, val in self.problemState['link_state'].iteritems():
                node0 = link['origin']
                node1 = link['destination']
                if link['bandwidth'] >= self.minBandwidth:
                    Links_list.append((node0, node1, link))
            G.add_edges_from(Links_list)
            # Partition the network
            self.ConnectedComponents = NetworkPartitioning.partition_by_latency(
                G, max_diameter=len(G.nodes()))
        elif mode == 'Hops':
            for link in self.ScenarioParameters['CommunicationNetwork']:
                node0 = link['origin']
                node1 = link['destination']
                _val = link.copy()
                _val['length'] = 1
                Links_list.append((node0, node1, _val))
            G.add_edges_from(Links_list)
            # Partition the network
            self.ConnectedComponents = NetworkPartitioning.partition_by_latency(
                G, max_diameter=self.maxHops, distance_label='length')
        elif mode == 'BandwidthHops':
            for link in self.ScenarioParameters['CommunicationNetwork']:
                node0 = link['origin']
                node1 = link['destination']
                if link['bandwidth'] >= self.minBandwidth:
                    Links_list.append((node0, node1, link))
            G.add_edges_from(Links_list)
            # Partition the network
            self.ConnectedComponents = NetworkPartitioning.partition_by_latency(
                G, max_diameter=self.maxHops)
        else:
            # Mode 'latency' is not supported for this I/O format
            raise NotImplementedError

        NetworkPartitioning.label_connected_components(
            G, self.ConnectedComponents, plot=False)
        self.G = G

        self.isPartitioned = True

        return self.ConnectedComponents

    def buildScheduler(self):
        ''' Creates the problem instance to be solved by the scheduler as a JSON file'''
        if self.isPartitioned is False:
            self.partition()

        ScenarioParameters = self.ScenarioParameters
        # problemState = self.problemState
        BaseStationName = self.BaseStationName
        # Build a sub-problem just for the agent's partition

        # Find where the agent is
        agent = ScenarioParameters['Agent']
        AgentNames = [agent]
        for component in self.ConnectedComponents:
            if agent in component:
                AgentNames = component
                break
        # print "Agent names in partition:", AgentNames, "All partitions:", self.ConnectedComponents
        self.AgentNames = AgentNames
        self.agent = agent

        PufferNames = list(AgentNames)
        if BaseStationName in PufferNames:
            PufferNames.remove(BaseStationName)

        # Build the bandwidth graph for the relevant connected component
        Thor_s = ScenarioParameters['Time']['TimeHorizon']
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
                    CommWindowsBandwidth[t][Agent1][Agent2] = 0
        for link in ScenarioParameters["CommunicationNetwork"]:
            Agent1 = link['origin']
            Agent2 = link['destination']
            for t in range(Thor):
                if (Agent1 in AgentNames) and (Agent2 in AgentNames):
                    CommWindowsBandwidth[t][Agent1][Agent2] = link['bandwidth']

        # Extract the science availability
        # TODO How many samples can you take within the given horizon
        achievable_science_in_horizon = 3

        pufferScienceAvailability = {}
        for puffer in PufferNames:
            in_science_zone = ScenarioParameters['AgentStates'][puffer]['in_science_zone']
            if "samples" in ScenarioParameters['AgentStates'][puffer].keys():
                puffer_samples = ScenarioParameters['AgentStates'][puffer]["samples"]
            else:
                puffer_samples = achievable_science_in_horizon
            pufferScienceAvailability[puffer] = puffer_samples * \
                in_science_zone

        # Here we specify the tasks that _each PUFFER_ should perform. We will assemble them later.

        # Reward for performing a task
        PufferTaskReward_housekeeping = {
            'short_range_image': ScenarioParameters['Tasks']['TaskReward']['short_range_image'],
            'vo_localization': ScenarioParameters['Tasks']['TaskReward']['vo_localization'],
            'plan_path': ScenarioParameters['Tasks']['TaskReward']['plan_path'],
            'send_drive_cmd': ScenarioParameters['Tasks']['TaskReward']['send_drive_cmd'],
        }

        PufferTaskReward_science = {
            'take_sample': ScenarioParameters['Tasks']['TaskReward']['take_sample'],
            'analyze_sample': ScenarioParameters['Tasks']['TaskReward']['analyze_sample'],
            'store_sample': ScenarioParameters['Tasks']['TaskReward']['store_sample'],
        }

        # pufferScienceReward = None  # This would allow us to override the task reward on a per-target basis

        # Which tasks are optional for performing a task

        PufferOptionalTask_housekeeping = {
            'short_range_image': False,
            'vo_localization': False,
            'plan_path': False,
            'send_drive_cmd': False,
        }

        PufferOptionalTask_science = {
            'take_sample': True,
            'analyze_sample': True,
            'store_sample': True,
        }

        # Size of the data products
        PufferProductSizes_housekeeping = {
            'short_range_image': ScenarioParameters['Tasks']['ProductsSize']['short_range_image'],
            'vo_localization': ScenarioParameters['Tasks']['ProductsSize']['vo_localization'],
            'plan_path': ScenarioParameters['Tasks']['ProductsSize']['plan_path'],
            'send_drive_cmd': ScenarioParameters['Tasks']['ProductsSize']['send_drive_cmd'],
        }

        PufferProductSizes_science = {
            'take_sample': ScenarioParameters['Tasks']['ProductsSize']['take_sample'],
            'analyze_sample': ScenarioParameters['Tasks']['ProductsSize']['analyze_sample'],
            'store_sample': ScenarioParameters['Tasks']['ProductsSize']['store_sample'],
        }

        pufferOwnHousekeepingTasks = [
            'short_range_image',
            'vo_localization',
            'plan_path',
            'send_drive_cmd',
        ]

        pufferOwnScienceTasks = [
            'take_sample',
            'analyze_sample',
            'store_sample',
        ]

        RelocatableTasks = [
            'vo_localization',
            'plan_path',
            'analyze_sample',
            'store_sample'
        ]

        # Dependency map for the PUFFERs. Recall that dependencies are in CNF.
        PufferDependencyList_housekeeping = {
            'short_range_image': [[]],
            'vo_localization': [['short_range_image']],
            'plan_path': [['vo_localization']],
            'send_drive_cmd': [['plan_path']]
        }

        PufferDependencyList_science = {
            'take_sample': [[]],
            'analyze_sample': [['take_sample']],
            'store_sample': [['analyze_sample']]
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
                #    Temp_PufferTaskReward.update({'{}:{}'.format(pufferName, scienceNo) + k: v for k, v in pufferScienceReward[pufferName][scienceNo].iteritems()})

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
            for task in pufferOwnHousekeepingTasks:
                tasktime = ScenarioParameters['AgentCapabilities']['ComputationTime'][pufferName].get(
                    task, InfeasibleTime)
                AllComputationTime.update(
                    {'{}:'.format(pufferName) + task: {pufferName: tasktime}})
                AllComputationLoad.update(
                    {'{}:'.format(pufferName) + task: {pufferName: 1.}})
                AllInitialInformation.update(
                    {'{}:'.format(pufferName) + task: {pufferName: False}})
            if BaseStationName in AgentNames:  # If the base station is in the group
                for task in pufferOwnHousekeepingTasks:
                    if task in RelocatableTasks:
                        tasktime = ScenarioParameters['AgentCapabilities']['ComputationTime'][BaseStationName].get(
                            task, InfeasibleTime)
                    else:
                        tasktime = InfeasibleTime
                    AllComputationTime['{}:'.format(
                        pufferName) + task][BaseStationName] = tasktime
                    AllComputationLoad['{}:'.format(pufferName) +
                                       task][BaseStationName] = 1.
                    AllInitialInformation['{}:'.format(
                        pufferName) + task][BaseStationName] = False

            for otherPuffer in PufferNames:
                if otherPuffer != pufferName:
                    for task in pufferOwnHousekeepingTasks:
                        if task in RelocatableTasks:
                            tasktime = ScenarioParameters['AgentCapabilities']['ComputationTime'][otherPuffer].get(
                                task, InfeasibleTime)
                        else:
                            tasktime = InfeasibleTime
                        AllComputationTime['{}:'.format(
                            pufferName) + task][otherPuffer] = tasktime
                        AllComputationLoad['{}:'.format(pufferName) +
                                           task][otherPuffer] = 1.
                        AllInitialInformation['{}:'.format(
                            pufferName) + task][otherPuffer] = False

            for scienceNo in range(pufferScienceAvailability[pufferName]):
                for task in pufferOwnScienceTasks:
                    tasktime = ScenarioParameters['AgentCapabilities']['ComputationTime'][pufferName].get(
                        task, InfeasibleTime)
                    AllComputationTime.update(
                        {'{}:{}'.format(pufferName, scienceNo) + task: {pufferName: tasktime}})
                    AllComputationLoad.update(
                        {'{}:{}'.format(pufferName, scienceNo) + task: {pufferName: 1.}})
                    AllInitialInformation.update(
                        {'{}:{}'.format(pufferName, scienceNo) + task: {pufferName: False}})
                if BaseStationName in AgentNames:  # If the base station is in the group
                    for task in pufferOwnScienceTasks:  # for task in pufferOwnScienceTasks:
                        if task in RelocatableTasks:
                            tasktime = ScenarioParameters['AgentCapabilities']['ComputationTime'][BaseStationName].get(
                                task, InfeasibleTime)
                        else:
                            tasktime = InfeasibleTime
                        AllComputationTime['{}:{}'.format(
                            pufferName, scienceNo) + task][BaseStationName] = tasktime
                        AllComputationLoad['{}:{}'.format(
                            pufferName, scienceNo) + task][BaseStationName] = 1.
                        AllInitialInformation['{}:{}'.format(
                            pufferName, scienceNo) + task][BaseStationName] = False

                for otherPuffer in PufferNames:
                    if otherPuffer != pufferName:
                        for task in pufferOwnScienceTasks:
                            if task in RelocatableTasks:
                                tasktime = ScenarioParameters['AgentCapabilities']['ComputationTime'][otherPuffer].get(
                                    task, InfeasibleTime)
                            else:
                                tasktime = InfeasibleTime
                            AllComputationTime['{}:{}'.format(
                                pufferName, scienceNo) + task][otherPuffer] = tasktime
                            AllComputationLoad['{}:{}'.format(
                                pufferName, scienceNo) + task][otherPuffer] = 1.
                            AllInitialInformation['{}:{}'.format(
                                pufferName, scienceNo) + task][otherPuffer] = False

        # Map from seconds to time steps
        #AllComputationTime = {k: int(np.ceil(float(v) / float(TimeStep))) for k, v in AllComputationTime.iteritems()}
        for task, val in AllComputationTime.items():
            for agent, tasktime in val.items():
                AllComputationTime[task][agent] = int(
                    np.ceil(float(tasktime) / float(TimeStep)))

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
                TaskRGBColor = np.array(colorsys.hsv_to_rgb(
                    TempPufferHSVColor[0] + TempTaskHSVColor[0],
                    TempPufferHSVColor[1] + TempTaskHSVColor[1],
                    TempPufferHSVColor[2] + TempTaskHSVColor[2])
                )
                AllTaskColors['{}:'.format(
                    pufferName) + PufferTask] = TaskRGBColor
                PufferTaskCounter += 1
            for PufferTask in PufferTaskList_science:
                TempTaskHSVColor = np.array(
                    [0., .2 + .8 * (1. - float(PufferTaskCounter) / float(NumPufferTasks - 1)), 0.])
                TaskRGBColor = np.array(colorsys.hsv_to_rgb(
                    TempPufferHSVColor[0] + TempTaskHSVColor[0],
                    TempPufferHSVColor[1] + TempTaskHSVColor[1],
                    TempPufferHSVColor[2] + TempTaskHSVColor[2])
                )
                for scienceNo in range(pufferScienceAvailability[pufferName]):
                    AllTaskColors['{}:{}'.format(
                        pufferName, scienceNo) + PufferTask] = TaskRGBColor  # plt_colors.hsv_to_rgb(TempPufferHSVColor + TempTaskHSVColor)
                PufferTaskCounter += 1

            PufferNo += 1

        self.AllTaskColors = AllTaskColors

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

        # if CPLEX_Available:
        #     FeasibleStartDict = self._createFeasibleSolution(Tasks, AgentCapabilities, CommWindowsBandwidth, PufferNames, pufferOwnHousekeepingTasks, Thor)
        #     MIP_vars = []
        #     MIP_vals = []
        #     for key in FeasibleStartDict:
        #         MIP_vars.append(key)
        #         MIP_vals.append(FeasibleStartDict[key])
        #     self.FeasibleSolution = [MIP_vars, MIP_vals]

        # Options = {'Packet Comms': False}  # Scheduler options/params

        # if CPLEX_Available:
        #     self.scheduler = MOSAICSolver.MOSAICCPLEXScheduler(Thor, AgentCapabilities, Tasks, CommWindowsBandwidth, self.TimeStep, Options, Verbose=False)
        #     self.scheduler.setTimeLimits(DetTicksLimit=self.solverDetTicksLimit, ClockTimeLimit=self.solverClockLimit)
        #     self.schedulerTerminator = self.scheduler.getOptimizationTerminator()
        # else:
        #     self.scheduler = MOSAICSolver.MOSAICGLPKScheduler(Thor, AgentCapabilities, Tasks, CommWindowsBandwidth, self.TimeStep, Options, Verbose=False)
        # # timeLimit = {'Seconds': self.solverTimeLimit, 'Ticks per second': self.ticksPerSecond}

        # self.scheduler.setSolverParameters(self.solverParameters)
        self.isSetUp = True

    def createJSON(self):
        if self.isSetUp is False:
            self.buildScheduler()

        # Time
        Thor_s = self.ScenarioParameters['Time']['TimeHorizon']
        TimeStep = self.TimeStep
        Time = {"Thor": Thor_s, "TimeStep": TimeStep}

        # Tasks
        MaxLatency = {}
        for task in self.Tasks.DependencyList.keys():
            MaxLatency[task] = {}
            for disjTasks in self.Tasks.DependencyList[task]:
                for predecessor in disjTasks:
                    MaxLatency[task][predecessor] = 999.
        Tasks = {
            "OptionalTasks": self.Tasks.OptionalTasks,
            "TaskReward": self.Tasks.TaskReward,
            "ProductsSize": self.Tasks.ProductsSize,
            "DependencyList": self.Tasks.DependencyList,
            "IncompatibleTasks": self.Tasks.IncompatibleTasks,
            "MaxLatency": MaxLatency,
        }

        # Agent capabilities
        # Energy cost is proportional to task duration
        EnergyCost = {}
        for task in self.AgentCapabilities.ComputationTime.keys():
            EnergyCost[task] = {}
            for agent in self.AgentCapabilities.ComputationTime[task].keys():
                EnergyCost[task][agent] = float(
                    self.AgentCapabilities.ComputationTime[task][agent]) / float(Thor_s)
        # Max computation load is 1 for all
        MaxComputationLoad = {}
        for agent in self.AgentNames:
            MaxComputationLoad[agent] = 1.
        # Link computational load in
        LinkComputationalLoadIn = {}
        LinkComputationalLoadOut = {}
        for agent1 in self.AgentNames:
            LinkComputationalLoadIn[agent1] = {}
            LinkComputationalLoadOut[agent1] = {}
            for agent2 in self.AgentNames:
                LinkComputationalLoadIn[agent1][agent2] = 1.
                LinkComputationalLoadOut[agent1][agent2] = 1.

        AgentCapabilities = {
            "ComputationTime": self.AgentCapabilities.ComputationTime,
            "ComputationLoad": self.AgentCapabilities.ComputationLoad,
            "EnergyCost": EnergyCost,
            "MaxComputationLoad": MaxComputationLoad,
            "LinkComputationalLoadIn": LinkComputationalLoadIn,
            "LinkComputationalLoadOut": LinkComputationalLoadOut,
            "InitialInformation": self.AgentCapabilities.InitialInformation,
        }
        # Comm network
        CommunicationNetwork = []
        for link in self.ScenarioParameters["CommunicationNetwork"]:
            Agent1 = link['origin']
            Agent2 = link['destination']
            if (Agent1 in self.AgentNames) and (Agent2 in self.AgentNames):
                CommunicationNetwork.append(link)
        # CommunicationNetwork = self.ScenarioParameters["CommunicationNetwork"]

        # Options
        Options = {}
        if 'CostFunction' in self.ScenarioParameters.keys():
            Options['CostFunction'] = self.ScenarioParameters['CostFunction']

        dictInput = {
            "Tasks": Tasks,
            "AgentCapabilities": AgentCapabilities,
            "CommunicationNetwork": CommunicationNetwork,
            "Time": Time,
            "Options": Options,
        }
        self.JSONInput = json.dumps(dictInput)
        return self.JSONInput

    def getTaskColors(self):
        if self.isSetUp is False:
            raise Exception(
                "Please set up the problem [problem.setUp()] before calling getTaskColors!")
        return self.AllTaskColors


def clean_up_schedule(schedule):
    if type(schedule) == str:
        schedule = json.loads(schedule)
    if schedule is None:
        return schedule

    PUFFER_task_names = [
        'short_range_image',
        'vo_localization',
        'plan_path',
        'send_drive_cmd',
        'take_sample',
        'analyze_sample',
        'store_sample',
    ]

    def find_task_name(long_task_name):
        for task_name in PUFFER_task_names:
            if long_task_name.find(task_name) >= 0:
                return task_name
        return None

    for task in schedule['tasks']:
        if task['name'] != 'transfer_data':
            task_names = task['name'].split(':')
            if task_names[-1] in PUFFER_task_names:
                task_name = task_names[-1]
            else:
                task_name = find_task_name(task['name'])
            task['name'] = task_name
        else:
            product_names = task['params']['data_type'].split(':')
            if product_names[-1] in PUFFER_task_names:
                product_name = product_names[-1]
            else:
                product_name = find_task_name(task['params']['data_type'])
            task['params']['data_type'] = product_name

    return schedule


if __name__ == "__main__":
    from mosaic_schedulers.common.examples.PUFFER import ScenarioGenerator as PUFFERScenarioGenerator
    from mosaic_schedulers.schedulers.tv_milp import MOSAICSolver
    from mosaic_schedulers.schedulers.ti_milp import MOSAICTISolver
    from mosaic_schedulers.common.plotting import MOSAICplotter
    import random

    random_input = True
    # Create an input
    if random_input:
        random.seed(1)
        ScenarioParams = PUFFERScenarioGenerator.create_scenario(
            num_puffers=2,
            base_station=True,
            science_zone_probability=.6,
            plot_scenario=False
        )
    else:
        # Alternatively, static input
        ScenarioParams = """
{
  "AgentCapabilities": {
    "EnergyCost": {
      "puffer1": {
        "send_drive_cmd": 0.06666666666666667, 
        "plan_path": 0.3333333333333333, 
        "short_range_image": 0.13333333, 
        "vo_localization": 0.3333333333333333, 
        "take_sample": 0.2, 
        "analyze_sample": 0.3333333333333333
      }, 
      "base_station": {
        "plan_path": 0.03333333333333333, 
        "vo_localization": 0.03333333333333333, 
        "store_sample": 0.003333333333333333, 
        "analyze_sample": 0.03333333333333333
      }
    }, 
    "ComputationTime": {
      "puffer1": {
        "send_drive_cmd": 0.1, 
        "plan_path": 10, 
        "short_range_image": 3, 
        "vo_localization": 10, 
        "take_sample": 5, 
        "analyze_sample": 10
      }, 
      "base_station": {
        "plan_path": 0.1, 
        "vo_localization": 1.0, 
        "store_sample": 0.1, 
        "analyze_sample": 1.0
      }
    }
  }, 
  "CommunicationNetwork": [
    {
      "origin": "puffer1", 
      "latency": 0.001, 
      "time_start": 831.2, 
      "destination": "puffer1", 
      "time_end": 1.7976931348623157e+308, 
      "bandwidth": 11.0, 
      "energy_cost": 0.0
    }, 
    {
      "origin": "puffer1", 
      "latency": 0.001, 
      "time_start": 831.2, 
      "destination": "base_station", 
      "time_end": 1.7976931348623157e+308, 
      "bandwidth": 10.0, 
      "energy_cost": 0.0
    }, 
    {
      "origin": "base_station", 
      "latency": 0.001, 
      "time_start": 831.2, 
      "destination": "puffer1", 
      "time_end": 1.7976931348623157e+308, 
      "bandwidth": 10.0,
      "energy_cost": 0.0
    }, 
    {
      "origin": "base_station", 
      "latency": 0.001, 
      "time_start": 831.2, 
      "destination": "base_station", 
      "time_end": 1.7976931348623157e+308, 
      "bandwidth": 11.0, 
      "energy_cost": 0.0
    }
  ], 
  "Tasks": {
    "DomainSpecific": {}, 
    "ProductsSize": {
      "send_drive_cmd": 0.1, 
      "plan_path": 0.1, 
      "vo_localization": 0.1, 
      "short_range_image": 8.0, 
      "store_sample": 0.1, 
      "take_sample": 15, 
      "analyze_sample": 1.0
    }, 
    "TaskReward": {
      "send_drive_cmd": 0, 
      "plan_path": 0, 
      "vo_localization": 0, 
      "short_range_image": 0, 
      "store_sample": 20, 
      "take_sample": 0, 
      "analyze_sample": 10
    }
  }, 
  "AgentStates": {
    "puffer1": {
      "in_science_zone": true, 
      "samples": 0
    }, 
    "base_station": {}
  }, 
  "Agent": "puffer1", 
  "Agents": [
    "puffer1", 
    "base_station"
  ], 
  "Time": {
    "CurrentTime": 1001, 
    "TimeHorizon": 60
  }, 
  "Options": {}, 
  "CostFunction": {
    "total_time": 0.5, 
    "energy": 0, 
    "total_task_reward": 1.0
  }
}
"""

    # Create the JSON
    Problem = MOSAICProblem(ScenarioParams)

    ConnectedComponents = Problem.partition()
    Problem.buildScheduler()
    JSONinput = Problem.createJSON()

    # Get the task colors
    TaskColors = Problem.getTaskColors()

    TaskColors = None

    # Test with a TV solver
    Scheduler = MOSAICSolver.JSONSolver(JSONinput)
    Schedule = Scheduler.schedule()
    MOSAICplotter.SchedulePlotter(Schedule, TaskColors)

    # # Test with default non-JSON solver to be sure
    # NonJSONSchedule = Problem.schedule()
    # print NonJSONSchedule
    # MOSAICSolver.SchedulePlotter(NonJSONSchedule,TaskColors)

    # Test with a TI solver
    TIScheduler = MOSAICTISolver.JSONSolver(JSONinput)
    TISchedule = TIScheduler.schedule()
    MOSAICplotter.TaskAllocationPlotter(TISchedule, TaskColors=TaskColors)
