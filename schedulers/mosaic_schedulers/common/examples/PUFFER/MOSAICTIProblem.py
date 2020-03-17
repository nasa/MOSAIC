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
from mosaic_schedulers.common.utilities import NetworkPartitioning
from mosaic_schedulers.common.examples.PUFFER import MOSAICProblem
from mosaic_schedulers.common.plotting import TaskAllocationPlotter

import networkx as nx
import numpy as np
import time
import json

try:
    import cplex
    CPLEX_Available = True
except:
    CPLEX_Available = False


class MOSAICTIProblem:
    """ An instance of the MOSAIC time-invariant scheduler.
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
            usage example in __main__
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

        self.TVProblem = MOSAICProblem.MOSAICProblem(self.problemState,
                                                     maxLatency=self.maxLatency,
                                                     minBandwidth=self.minBandwidth,
                                                     solverClockLimit=self.solverClockLimit,
                                                     solverDetTicksLimit=self.solverDetTicksLimit,
                                                     solverParameters=self.solverParameters,
                                                     )

    def partition(self, mode='Bandwidth'):
        # Partition the network
        self.ConnectedComponents = self.TVProblem.partition(
            mode)  # Partition the network
        self.isPartitioned = True

    def buildScheduler(self):
        # Build the TV scheduler
        self.TVProblem.buildScheduler()

        # Convert the scheduler to the time-invariant version

        # Agent and tasks list
        TasksList = list(self.TVProblem.Tasks.ProductsSize.keys())
        AgentsList = list(
            self.TVProblem.AgentCapabilities.ComputationTime[TasksList[0]].keys())
        # Dependency list
        DepList = {}
        for task in self.TVProblem.Tasks.DependencyList.keys():
            if self.TVProblem.Tasks.DependencyList[task]:
                DepList[task] = []
                for dep in self.TVProblem.Tasks.DependencyList[task]:
                    if dep:
                        assert len(
                            dep) == 1, 'Disjunctive prerequirements are not supported'
                        DepList[task] += [dep[0]]
        # Products bandwidth
        PrBandwidth = {}
        for task in self.TVProblem.Tasks.ProductsSize.keys():
            # Average bandwidth to finish by the end of the horizon
            PrBandwidth[task] = float(
                self.TVProblem.Tasks.ProductsSize[task])/float(self.TVProblem.Thor)
        # Max. latency
        MaxLat = {}
        for task in DepList.keys():
            MaxLat[task] = {}
            for predecessor in DepList[task]:
                MaxLat[task][predecessor] = 999.
        # Computation load and energy cost
        CompLoad = {}
        EnergyCost = {}
        for task in self.TVProblem.AgentCapabilities.ComputationTime.keys():
            for agent in self.TVProblem.AgentCapabilities.ComputationTime[task].keys():
                if agent not in CompLoad.keys():
                    CompLoad[agent] = {}
                    EnergyCost[agent] = {}
                CompLoad[agent][task] = float(
                    self.TVProblem.AgentCapabilities.ComputationTime[task][agent]) / float(self.TVProblem.Thor)
                EnergyCost[agent][task] = CompLoad[agent][task]
        MaxComputationalLoad = {}
        for agent in AgentsList:
            MaxComputationalLoad[agent] = 1.
        # Link variables
        LinkComputationalLoadIn = {}
        LinkComputationalLoadOut = {}
        CommWindowsBandwidth = {}
        CommWindowsLatency = {}
        CommWindowsEnergyCost = {}

        for Agent1 in AgentsList:
            CommWindowsBandwidth[Agent1] = {}
            LinkComputationalLoadIn[Agent1] = {}
            LinkComputationalLoadOut[Agent1] = {}
            CommWindowsLatency[Agent1] = {}
            CommWindowsEnergyCost[Agent1] = {}
            for Agent2 in AgentsList:
                if (Agent1, Agent2) in self.problemState['link_state'].keys():
                    CommWindowsBandwidth[Agent1][Agent2] = self.problemState['link_state'][(
                        Agent1, Agent2)]['bandwidth']
                else:
                    CommWindowsBandwidth[Agent1][Agent2] = 0
                LinkComputationalLoadIn[Agent1][Agent2] = 0.02
                LinkComputationalLoadOut[Agent1][Agent2] = 0.02
                CommWindowsLatency[Agent1][Agent2] = 0.0001
                CommWindowsEnergyCost[Agent1][Agent2] = 0.05

        # Now create the time-invariant problem
        AgentCapabilities = MOSAICTISolver.MILPAgentCapabilities(
            ComputationLoad=CompLoad,
            EnergyCost=EnergyCost,
            MaxComputationLoad=MaxComputationalLoad,
            LinkComputationalLoadIn=LinkComputationalLoadIn,
            LinkComputationalLoadOut=LinkComputationalLoadOut
        )

        Tasks = MOSAICTISolver.MILPTasks(
            OptionalTasks=self.TVProblem.Tasks.OptionalTasks,
            TaskReward=self.TVProblem.Tasks.TaskReward,
            ProductsBandwidth=PrBandwidth,
            ProductsDataSize=self.TVProblem.Tasks.ProductsSize,
            DependencyList=DepList,
            MaxLatency=MaxLat,
        )
        Tasks.validate()

        CommunicationNetwork = MOSAICTISolver.CommunicationNetwork(
            Bandwidth=CommWindowsBandwidth,
            Latency=CommWindowsLatency,
            EnergyCost=CommWindowsEnergyCost
        )
        Options = {}
        Options['cost'] = 'optional'
        self.AgentCapabilities = AgentCapabilities
        self.Tasks = Tasks
        self.CommunicationNetwork = CommunicationNetwork

        if CPLEX_Available:
            self.scheduler = MOSAICTISolver.MOSAICCPLEXSolver(
                self.AgentCapabilities, self.Tasks, self.CommunicationNetwork, Options)
        else:
            self.scheduler = MOSAICTISolver.MOSAICGLPKSolver(
                self.AgentCapabilities, self.Tasks, self.CommunicationNetwork, Options)
            # self.schedulerTerminator = self.scheduler.getOptimizationTerminator()

        self.AllTaskColors = self.TVProblem.AllTaskColors

        self.isSetUp = True

    def _createFeasibleSolution(self):
        raise NotImplementedError

    # def getOptimizationTerminator(self):
    #     if self.isSetUp is not True:
    #         raise Exception("Problem is not set up yet, terminator not available")
    #     return self.schedulerTerminator

    def schedule(self):
        setup_start_time = time.process_time()
        if self.isSetUp is False:
            self.buildScheduler()
        # # Set feasible solution
        # self.scheduler.setUp()
        # try:
        #     self.scheduler.setMIPStart(self.FeasibleSolution, self.scheduler.cpx.MIP_starts.effort_level.check_feasibility)
        # except cplex.exceptions.CplexSolverError:
        #     print("WARNING: default feasible solution is problematic. This should never happen")
        #     pass
        setup_end_time = time.process_time() - setup_start_time
        remaining_solver_time = self.solverClockLimit - setup_end_time
        if remaining_solver_time < 0:
            print("ERROR: Not enough time to solve - timeout while building the problem. Consider increasing ClockTimeLimit")
            return
        self.scheduler.setTimeLimits(
            ClockTimeLimit=remaining_solver_time, DetTicksLimit=self.solverDetTicksLimit)
        self.scheduler.schedule()
        self.Schedule = self.scheduler.formatToBenchmarkIO()
        self.isScheduled = True

        return self.Schedule

    def getTaskColors(self):
        if self.isSetUp is False:
            raise Exception(
                "Please set up the problem [problem.setUp()] before calling getTaskColors!")
        return self.AllTaskColors


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
    MProblem = MOSAICTIProblem(problemState,
                               # Disconnect agents if latency is higher than this value and partition mode is 'Latency'
                               maxLatency=1e-7,
                               minBandwidth=6,    # Disconnect agents if bandwidth is lower than this value and partition mode is 'Bandwidth'
                               # Emergency stop: stop optimization if it takes longer than this many seconds (non-deterministic)
                               solverClockLimit=20,
                               # Deterministic time limit: Stop optimization if it takes longer than this many ticks
                               solverDetTicksLimit=10000,
                               solverParameters=solverParameters,  # Parameters for the MIP solver
                               )

    NodesLocations = {}
    NodesLocations['base_station'] = {'x': 0., 'y': 0.}
    NodesLocations['puffer1'] = {'x': 0., 'y': 1.}
    NodesLocations['puffer2'] = {'x': 1., 'y': 0.}
    NodesLocations['puffer3'] = {'x': 1., 'y': 1.}

    # Partition the network
    ConnectedComponents = MProblem.partition(
        mode='Bandwidth')  # Partition the network
    # Build the scheduler
    MProblem.buildScheduler()
    # # Get an object to terminate the optimization asynchronously
    # OptimizationTerminator = MProblem.getOptimizationTerminator()
    # # Call the scheduler, get back a JSON solution
    Schedule = MProblem.schedule()
    # '''
    # Get the problem state (to know why it stopped). A list of stopping criteria
    # and corresponding exit codes can be found at:
    # https://www.ibm.com/support/knowledgecenter/SSSA5P_12.8.0/ilog.odms.cplex.help/refcallablelibrary/macros/homepagesolutionstatusMIP.html
    # Relevant stopping criteria:
    # Deterministic time limit, feasible (synchronization guaranteed): "CPXMIP_DETTIME_LIM_FEAS", 131
    # Deterministic time limit, INfeasible : "CPXMIP_DETTIME_LIM_INFEAS", 132
    # Nondeterministic time limit, feasible (synchronization not guaranteed): "CPXMIP_TIME_LIM_FEAS", 107
    # Nondeterministic time limit, INfeasible: "CPXMIP_TIME_LIM_INFEAS", 108
    # Aborted by user "CPX_STAT_ABORT_USER", 13
    # '''
    solverState = MProblem.scheduler.getSolverState()
    # # Get the solution in a format that can be used to seed future MILPs
    # MIPStart = MProblem.scheduler.getMIPStart()
    # MProblem.scheduler.setMIPStart(MIPStart)

    print(Schedule)
    # Get task colors (for plotting)
    TaskColors = MProblem.getTaskColors()
    # Plot the schedule
    TaskAllocationPlotter(Schedule, NodesLocations,
                          TaskColors, BandwidthScale=10, show=True, save=False)
