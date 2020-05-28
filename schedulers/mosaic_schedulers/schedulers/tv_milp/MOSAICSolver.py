"""
A module that implements the MOSAIC MILP scheduling algorithm.
"""

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


# pylint:disable=E1101
import pulp
import numpy as np
import unittest
import time
import copy
try:
    import cplex  # This will be problematic on ARM
    CPLEXAvailable = True
except:
    CPLEXAvailable = False
import json
try:
    # Install pyglpk by bradforboyle. Will not work with Python-GLPK.
    import glpk
    GLPKAvailable = True
except:
    GLPKAvailable = False
try:
    import pyscipopt as scip
    SCIPAvailable = True
except:
    SCIPAvailable = False
from mosaic_schedulers.common.utilities.contact_plan_handler import compute_time_varying_bandwidth
from mosaic_schedulers.common.utilities.contact_plan_handler import compute_time_invariant_bandwidth_legacy
import sys
import os


class MILPTasks:
    """A class containing a representation of the tasks for the MILP scheduler.
    
    :param OptionalTasks: a dict with Tasks as keys. OptionalTasks[Task] is True if
                          Task is optional and False otherwise.
    :type OptionalTasks: dict
    :param TaskReward: a dict with Tasks as keys. TaskReward[Task] is the reward
                       obtained for completing Task, a float.
    :type OptionalTasks: dict
    :param ProductsSize: a dict with Tasks as keys. PS[Task] is the size of the
                         products of Task (in storage units), a float.
    :type ProductsSize: dict
    :param DependencyList: a dict with Tasks as keys. DL[Task] expresses the
                           pre-requisites  for each Task is conjunctive normal form. That is,
                           DependencyList[Task] is a list. Each entry of the List contains a
                           number of predecessor Tasks.
                           For each entry in DependencyList[Task][i], at least one task must
                           be performed.
    :type DependencyList: dict
    :param OptionalTasks: a list of lists. IT[j] is a list of  tasks that are incompatible
                          with each other (that is, only one can be executed).
    :type OptionalTasks: list
    """

    def __init__(self, OptionalTasks={},
                 TaskReward={},
                 ProductsSize={},
                 DependencyList={},
                 IncompatibleTasks={},
                 ):
        self.OptionalTasks = OptionalTasks
        self.TaskReward = TaskReward
        self.ProductsSize = ProductsSize
        self.DependencyList = DependencyList
        self.IncompatibleTasks = IncompatibleTasks

        assert set(OptionalTasks.keys()) == set(TaskReward.keys())
        assert set(OptionalTasks.keys()) == set(ProductsSize.keys())
        assert set(DependencyList.keys()) <= (set(OptionalTasks.keys()))


class MILPAgentCapabilities:
    """A class containing a representation of the agent capabilities for the
    MILP scheduler.
    
    :param ComputationTime: a dictionary with keys Task, Agent. The value of
      CT[Task][Agent] is the time (in time steps) required for Agent to
      complete Tasks, an int.
    :type ComputationTime: dict
    :param ComputationLoad: a dictionary with keys Task, Agent. The value of
      CT[Task][Agent] is the fraction of Agent's computational resources
      required to complete Task, a float between 0 and 1.
    :type ComputationLoad: dict
    :param  InitialInformation: a dictionary with keys Task, Agent. II[Task][Agent]
      is a Bool. It is true iff Agent knows the output of Task at time t=0.
      This can be helpful to specify that only one agent can do a given task
    :type InitialInformation: dict
    :param EnergyCost: a dictionary with keys Task, Agent. EnergyCost[T][A] is the
      energy cost when agent A computes task T.
    :type EnergyCost: bool
    :param CommEnergyCost: a dictionary witk keys Time, Sender, Receiver. CET[t][i][j]
      is the energy cost to send one bit of information from i to j at time t.
      The actual energy cost is computed as CET[t][i][j]\*Bandwidth[t][i][j]\*
      \*sum_m C[i][j][m][t]
    :type CommEnergyCost: bool
    """

    def __init__(self, ComputationTime={},
                 ComputationLoad={},
                 InitialInformation={},
                 EnergyCost={},
                 CommEnergyCost={}
                 ):
        self.ComputationTime = ComputationTime
        self.ComputationLoad = ComputationLoad
        self.InitialInformation = InitialInformation
        self.EnergyCost = EnergyCost
        self.CommEnergyCost = CommEnergyCost
        if self.EnergyCost == {}:
            Tasks = self.ComputationTime.keys()
            for m in Tasks:
                self.EnergyCost[m] = {}
                Agents = self.ComputationTime[m].keys()
                for i in Agents:
                    self.EnergyCost[m][i] = 0.


class CommunicationNetwork:
    """ A class containing a representation of the communication network.
    
    :param CommWindowsBandwidth: a collection of communication windows.
        CommWindows is a dictionary with keys [0...Thor-1], [Agent], [Agent].
        CommWindows[Time][Agent1][Agent2] is the bandwidth available
        to communicate between Agent1 and Agent2 at Time (in bits per
        unit time).
    :type CommWindowsBandwidth: dict
    :param  CommEnergyCost: a dictionary witk keys Time, Sender, Receiver.
        CET[t][i][j] is the energy cost to send one bit of information from
        i to j at time t. The actual energy cost is computed as
        CET[t][i][j]*Bandwidth[t][i][j]*sum_m C[i][j][m][t]
    :type CommEnergyCost: dict
    """

    def __init__(self, CommWindowsBandwidth={}, CommEnergyCost=None):

        self.CommWindowsBandwidth = CommWindowsBandwidth
        if CommEnergyCost is None:
            CommEnergyCost = {}
            for t in CommWindowsBandwidth.keys():
                CommEnergyCost[t] = {}
                for i in CommWindowsBandwidth[t].keys():
                    CommEnergyCost[t][i] = {}
                    for j in CommWindowsBandwidth[t][i].keys():
                        CommEnergyCost[t][i][j] = 0.
        self.CommEnergyCost = CommEnergyCost


class JSONSolver:
    """
    The main interface to solve MOSAIC scheduling problems

    :param JSONProblemDescription: a JSON description of the problem. See the :doc:`API` for a detailed description. Defaults to ''
    :type JSONProblemDescription: str, optional
    :param solver: the solver to use. Should be 'GLPK', 'CPLEX', 'PuLP' or 'SCIP', defaults to 'CPLEX'
    :type solver: str, optional
    :param Verbose: whether to print status and debug messages. Defaults to False
    :type Verbose: bool, optional
    :param TimeLimit: a time limit after which to stop the optimizer. Defaults to None (no time limit). Note that certain solvers (GLPK) 
                      may not honor the time limit.
    :type TimeLimit: float, optional
    :raises ValueError: if the JSONProblemDescription is not valid.
    """
    def __init__(self,
                 JSONProblemDescription='',
                 solver='CPLEX',
                 Verbose=False,
                 TimeLimit=None):

        JSONInput = json.loads(JSONProblemDescription)
        if solver not in ['GLPK', 'CPLEX', 'PuLP', 'SCIP']:
            if CPLEXAvailable:
                print("WARNING: invalid solver. Defaulting to CPLEX")
                solver = "CPLEX"
            else:
                print("WARNING: invalid solver. Defaulting to GLPK")
                solver = "GLPK"
        if solver == "CPLEX" and (CPLEXAvailable is False):
            print("WARNING: CPLEX not available. Switching to GLPK")
            solver = "GLPK"

        if solver == "SCIP" and (SCIPAvailable is False):
            print("WARNING: SCIP not available. Switching to GLPK")
            solver = "GLPK"

        if solver == "GLPK" and (GLPKAvailable is False):
            print("WARNING: GLPK not available. Aborting.")
            return

        # Timing
        Thor_s = JSONInput["Time"]["Thor"]
        TimeStep = JSONInput["Time"]["TimeStep"]
        Thor = int(float(Thor_s) / float(TimeStep))

        # Tasks
        Tasks = MILPTasks(
            OptionalTasks=JSONInput["Tasks"]["OptionalTasks"],
            TaskReward=JSONInput["Tasks"]["TaskReward"],
            ProductsSize=JSONInput["Tasks"]["ProductsSize"],
            DependencyList=JSONInput["Tasks"]["DependencyList"],
            IncompatibleTasks=JSONInput["Tasks"]["IncompatibleTasks"]
        )

        AllComputationTime = JSONInput["AgentCapabilities"]["ComputationTime"]
        for task, val in AllComputationTime.items():
            for agent, tasktime in val.items():
                AllComputationTime[task][agent] = int(
                    np.ceil(float(tasktime) / float(TimeStep)))

        # Agent capabilities
        AgentCapabilities = MILPAgentCapabilities(
            ComputationTime=AllComputationTime,
            ComputationLoad=JSONInput["AgentCapabilities"]["ComputationLoad"],
            InitialInformation=JSONInput["AgentCapabilities"]["InitialInformation"],
            EnergyCost=JSONInput["AgentCapabilities"]["EnergyCost"]
        )

        # Options
        Options = JSONInput["Options"]
        if 'Packet Comms' not in Options.keys():
            Options['Packet Comms'] = False
        if 'CostFunction' in JSONInput.keys():
            Options['CostFunction'] = JSONInput['CostFunction']
        bandwidth_mode = JSONInput.get("BandwidthMode", 'time_varying')

        # Communication bandwidth
        if bandwidth_mode == 'time_varying':
            CommWindows, CommEnergyCost = compute_time_varying_bandwidth(
                AgentNames=JSONInput["AgentCapabilities"]["MaxComputationLoad"].keys(
                ),
                TVBandwidth=JSONInput["CommunicationNetwork"],
                time_horizon=Thor_s,
                time_step=TimeStep
            )
        elif bandwidth_mode == 'time_invariant':
            CommWindows, CommEnergyCost = compute_time_invariant_bandwidth_legacy(
                AgentNames=JSONInput["AgentCapabilities"]["MaxComputationLoad"].keys(
                ),
                TVBandwidth=JSONInput["CommunicationNetwork"],
                time_horizon=Thor_s,
                time_step=TimeStep
            )
        else:
            raise ValueError(
                "Bandwidth mode \"{}\" not recognized!".format(bandwidth_mode))
        CommNetwork = CommunicationNetwork(
            CommWindowsBandwidth=CommWindows,
            CommEnergyCost=CommEnergyCost
        )

        if solver == "CPLEX":
            self.Scheduler = MOSAICCPLEXScheduler(
                Thor,
                AgentCapabilities,
                Tasks,
                CommNetwork,
                TimeStep,
                Options,
                Verbose
            )
            if TimeLimit is not None:
                self.Scheduler.setTimeLimits(ClockTimeLimit=TimeLimit)
        elif solver == "GLPK":
            self.Scheduler = MOSAICGLPKScheduler(
                Thor,
                AgentCapabilities,
                Tasks,
                CommNetwork,
                TimeStep,
                Options,
                Verbose
            )
            if TimeLimit is not None:
                self.Scheduler.setTimeLimits(ClockTimeLimit=TimeLimit)
        elif solver == "PuLP":
            self.Scheduler = MOSAICPULPScheduler(
                Thor,
                AgentCapabilities,
                Tasks,
                CommNetwork,
                TimeStep,
                Options,
                Verbose
            )
        elif solver == "SCIP":
            self.Scheduler = MOSAICSCIPScheduler(
                Thor,
                AgentCapabilities,
                Tasks,
                CommNetwork,
                TimeStep,
                Options,
                Verbose
            )
            if TimeLimit is not None:
                self.Scheduler.setTimeLimits(ClockTimeLimit=TimeLimit)

    def schedule(self):
        self.timeline = self.Scheduler.schedule()
        self.output = self.Scheduler.formatOutput(version=2)
        return self.output


class MOSAICMILPScheduler:
    """
    .. warning ::
       
       For an easier-to-use and more consistent interface, you most likely want to call
       :class:`JSONSolver` instead of this class and its subclasses.

    Abstract implementation of a MILP solution to the MOSAIC scheduler.
    Subclassed by the solvers :class:`MOSAICCPLEXScheduler`, :class:`MOSAICSCIPScheduler`,
    :class:`MOSAICPULPScheduler`, and :class:`MOSAICGLPKScheduler`.
    You probably want to call :class:`JSONSolver` instead of these.

    Usage:
    
    * :func:`schedule`: creates the optimization problem, solves it, and returns a JSON representation of the solution (in non-standard format)
    
    * :func:`getOptimizationTerminator`: returns a function that can be called to stop the optimization process asynchronously
    
    
    For a detailed description of the MILP problem we solve, see the IJRR
    manuscript.

    :param Thor: the time horizon of the optimization, an int.
    :type Thor: int
    :param AgentCapabilities: detailing what the agents can do
    :type AgentCapabilities: MOSAICSolver.MILPAgentCapabilities
    :param Tasks: detailing the tasks that must be achieved
    :type Tasks: MOSAICSolver.MILPTasks
    :param CommunicationNetwork: detailing the communication network availability
    :type CommunicationNetwork: MOSAICSolver.CommunicationNetwork
    :param TimeStep: The duration (in seconds) of a discrete time step.
    :type TimeStep: float
    :param Options: additional solver-specific options, defaults to {}
    :type Options: dict, optional
    :param Verbose: if True, prints status and debug messages. Defaults to False
    :type Verbose: bool, optional
    """
    def __init__(self,
                 Thor,
                 AgentCapabilities,
                 Tasks,
                 CommunicationNetwork,
                 TimeStep,
                 Options={},
                 Verbose=False):
        self.Thor = Thor
        self.AgentCapabilities = AgentCapabilities
        self.Tasks = Tasks
        self.CommWindowsBandwidth = CommunicationNetwork.CommWindowsBandwidth
        self.CommEnergyCost = CommunicationNetwork.CommEnergyCost
        self.Options = Options
        self.TimeStep = TimeStep
        self.Verbose = Verbose
        self.problemIsSetUp = False
        self.problemIsSolved = False
        self.problemIsFeasible = False
        self.JSONIsFormed = False
        self.solverSpecificInit()

        self.TaskAssignment = None
        self.CommSchedule = None
        self.RawOutput = None

    def solverSpecificInit(self):
        ''' A place to define solver-specific initializations in subclasses '''
        pass

    def _FindMaxStartCommTime(self, Agent1, Agent2, Task, Time):
        ''' Helper function to find the earliest time a communication could have started '''
        # We look for max tau such that sum_{tt=tau}^{Time-1} CommBandwidth(tt)>=TaskSize.
        # First we note that sum_{tt=tau}^{Time-1} CommBandwidth(tt) = CumCommBandwidth(Time)-CumCommBandwidth(tau) (note that the definition of CUmCommBandwidth below is "shifted by one" for this reason)
        # Next, we sweep ts from 0 to Time and check the property. We find the first time that fails the test above and return ts-1. If even t=0 fails the test, well, there is no way to get done by t. So we return None
        for ts in range(Time + 1):
            if self.CumulativeCommBandwidth[Time][Agent1][Agent2] - self.CumulativeCommBandwidth[ts][Agent1][Agent2] < self.Tasks.ProductsSize[Task]:
                # Recall that we get the bandwidth from the beginning of ts until the end of T. To put it differently, the delta above is the delta from ts+1 until T
                tstart = ts - 1
                # print(tstart,'is the latest start time for task ',Task ,'to be done by time', Time, ' from agent ',Agent1,' to agent ',Agent2)
                if tstart == -1:
                    tstart = None
                return tstart
        return Time

    def _FindEndCommTime(self, Agent1, Agent2, Task, Time):
        ''' Helper function to find the end time for a given communication '''
        for ts in range(Time, self.Thor + 1):
            if self.CumulativeCommBandwidth[ts][Agent1][Agent2] - self.CumulativeCommBandwidth[Time][Agent1][Agent2] >= self.Tasks.ProductsSize[Task]:
                return ts
        # If there is no way this is done by the end of the horizon (should probably not happen)
        return None

    def _ComputeCumulativeCommBandwidth(self):
        ''' Helper function to find the total available number of bits that
        can be transmitted between two agents in a given time interval '''
        # Note that this is a _shifted_ cumsum with CCB(0)=0 and CCB(t)=sum_{tau=0}^{t-1} CB(tau).
        TaskNames = list(self.Tasks.OptionalTasks.keys())
        AgentNames = list(
            self.AgentCapabilities.ComputationTime[TaskNames[0]].keys())
        self.CumulativeCommBandwidth = {}
        for t in [0]:
            self.CumulativeCommBandwidth[t] = {}
            for Agent1 in AgentNames:
                self.CumulativeCommBandwidth[t][Agent1] = {}
                for Agent2 in AgentNames:
                    self.CumulativeCommBandwidth[t][Agent1][Agent2] = 0
        for t in range(1, self.Thor + 1):
            self.CumulativeCommBandwidth[t] = {}
            for Agent1 in AgentNames:
                self.CumulativeCommBandwidth[t][Agent1] = {}
                for Agent2 in AgentNames:
                    self.CumulativeCommBandwidth[t][Agent1][Agent2] = self.CommWindowsBandwidth[t -
                                                                                                1][Agent1][Agent2] + self.CumulativeCommBandwidth[t - 1][Agent1][Agent2]

    def _verbprint(self, _str):
        ''' Helper function to only print if the verbose flag is set'''
        if self.Verbose:
            print(_str)

    def formatToJSON(self):
        if self.problemIsSolved is False:
            self._verbprint("Calling solver")
            self.solve()
        if self.problemIsFeasible is False:
            self.Timeline = None
            self._verbprint("Problem infeasible!")
            return None
        TasksList = list(self.Tasks.OptionalTasks.keys())
        # AgentsList = list(self.AgentCapabilities.ComputationLoad[TasksList[0]].keys())
        TaskSchedule = self.TaskAssignment['TaskSchedule']
        TaskAgent = self.TaskAssignment['TaskAgent']
        ComputationTime = self.AgentCapabilities.ComputationTime
        TimeStep = self.TimeStep
        CommSchedule = self.CommSchedule

        # Create timeline
        # Timeline = pd.DataFrame(
        #     columns=['Agent', 'Start time', 'Duration', 'Task type', 'Task'])
        Timeline = []
        # Add tasks
        for taskid in TasksList:
            if (TaskAgent[taskid] is not None) and (TaskSchedule[taskid] is not None):
                _agent = TaskAgent[taskid]
                _time = float(TaskSchedule[taskid]) * TimeStep
                _duration = float(ComputationTime[taskid][_agent]) * TimeStep
                _name = taskid
                Timeline.append({'Agent': _agent, 'Start time': _time, 'Duration': _duration,
                                 'Task type': 'Compute', 'Task': _name})

        # Add communication
        for comm in CommSchedule:
            _agent = comm[0]
            _time = float(comm[3]) * TimeStep
            _taskname = comm[2]
            _receiver = comm[1]
            if len(comm) > 4:
                _duration = float(comm[4] - comm[3]) * TimeStep
            else:
                _duration = TimeStep

            CommTask = {'Transmitter': _agent,
                        'Receiver': _receiver, 'Product': _taskname}

            Timeline.append({'Agent': _agent, 'Start time': _time, 'Duration': _duration,
                             'Task type': 'Communicate', 'Task': CommTask})
            Timeline.append({'Agent': _receiver, 'Start time': _time, 'Duration': _duration,
                             'Task type': 'Communicate', 'Task': CommTask})

        # Timeline.sort_values(by='Start time', inplace=True)
        # Timeline.reset_index(inplace=True, drop=True)
        Timeline.sort(key=lambda row: row['Start time'])

        self.Timeline = json.dumps(Timeline)
        self.JSONIsFormed = True
        return self.Timeline

    def _formatToBenchmarkIO(self):
        """ Formats the problem in the format defined in the :doc:`API`"""
        tasks_output = []
        ''' Formats the scheduler output to JSON '''
        if self.problemIsSolved is False:
            self._verbprint("Calling solver")
            self.solve()
        if self.problemIsFeasible is False:
            self.Timeline = None
            self._verbprint("Problem infeasible!")
            return None
        TasksList = list(self.Tasks.OptionalTasks.keys())
        # AgentsList = list(self.AgentCapabilities.ComputationLoad[TasksList[0]].keys())
        TaskSchedule = self.TaskAssignment['TaskSchedule']
        TaskAgent = self.TaskAssignment['TaskAgent']
        ComputationTime = self.AgentCapabilities.ComputationTime
        TimeStep = self.TimeStep
        CommSchedule = self.CommSchedule

        for taskid in TasksList:
            if (TaskAgent[taskid] is not None) and (TaskSchedule[taskid] is not None):
                _agent = TaskAgent[taskid]
                _time = float(TaskSchedule[taskid]) * TimeStep
                _duration = float(ComputationTime[taskid][_agent]) * TimeStep
                _name = taskid
                task_output = {
                    "duration": _duration,
                    "start_time": _time,
                    "id": _name,
                    "name": _name,
                    "params": {
                        "agent": _agent,
                    },
                }
                tasks_output.append(task_output)
        for comm in CommSchedule:
            _agent = comm[0]
            _time = float(comm[3]) * TimeStep
            _taskname = comm[2]
            _receiver = comm[1]
            if len(comm) > 4:
                _duration = float(comm[4] - comm[3]) * TimeStep
            else:
                _duration = TimeStep
            task_output_tx = {
                "duration": _duration,
                "start_time": _time,
                "id": "transfer_data",
                "name": "transfer_data",
                "params": {
                    "transmitter": _agent,
                    "data_type": _taskname,
                    "agent": _agent,
                    "receiver": _receiver,
                    # TODO
                    "bandwidth": self.CommWindowsBandwidth[comm[3]][_agent][_receiver],
                },
            }
            tasks_output.append(task_output_tx)
            task_output_rx = copy.deepcopy(task_output_tx)
            task_output_rx["params"]["agent"] = task_output_rx["params"]["receiver"]
            tasks_output.append(task_output_rx)

        def sort_by_time(val):
            return val["start_time"]
        tasks_output.sort(key=sort_by_time)
        out_dict = {"tasks": tasks_output}
        return json.dumps(out_dict)

    def formatOutput(self, version=2):
        """
        Formats output.

        :param version: V1: just a pass-through for formatToJSON.
                        V2 (default): the standard I/O format defined in the :doc:`API`. 
        :type version: int, optional
        :return: JSON output
        :rtype: str
        """
        if version == 1:
            if not self.JSONIsFormed:
                self.formatToJSON()
            return self.Timeline
        elif version == 2:
            return self._formatToBenchmarkIO()

    def getRawOutput(self):
        """ Return the raw problem output from the solver """
        if self.problemIsSolved is False:
            self.solve()
        return self.TaskAssignment, self.CommSchedule, self.RawOutput

    def schedule(self):
        """ Set up the problem, solve it, and return the solution (in an older, non-JSON I/O format) """
        self.setUp()
        self.solve()
        if self.problemIsFeasible is False:
            print("Problem infeasible!")
            return None
        self.formatToJSON()
        return self.Timeline

    def setTimeLimits(self, DetTicksLimit, ClockTimeLimit):
        '''
        Sets a time limit for the overall process. May be ignored by certain
        solvers.
        '''
        raise NotImplementedError

    def setSolverParameters(self, Properties):
        '''
        Passes a dictionary of parameters to the solver
        '''
        raise NotImplementedError

    def setUp(self):
        ''' The problem is set up here. At this time, the entire setup is repeated for each solver. '''
        raise NotImplementedError

    def solve(self):
        ''' The problem is solved here. '''
        raise NotImplementedError

    def setMIPStart(self, MIPStart):
        ''' Certain solvers allow the MIP to be "seeded" with a MIP start. '''
        raise NotImplementedError

    def getSolverState(self):
        ''' This function allows extracting the internal MIP state for warm-starting. '''
        raise NotImplementedError

    def getProblem(self):
        """ Return the solver-specific problem object """
        raise NotImplementedError

    def getOptimizationTerminator(self):
        ''' Gets an object that can be called to stop the optimization process '''
        raise NotImplementedError


class MOSAICCPLEXScheduler(MOSAICMILPScheduler):
    """
    An implementation of the MILP scheduler that uses the IBM ILOG CPLEX solver.
    """

    def solverSpecificInit(self):
        if CPLEXAvailable is False:
            print("CPLEX not available. This will not work")
            return
        # Create the CPLEX problem
        self.cpx = cplex.Cplex()
        self.cpx.objective.set_sense(self.cpx.objective.sense.maximize)
        self.aborter = self.cpx.use_aborter(cplex.Aborter())
        if self.Options['Packet Comms'] is not True:
            self._ComputeCumulativeCommBandwidth()

    def setUp(self):
        if self.problemIsSetUp:
            return
        SetupStartTime = time.time()

        # This used to be a part of a larger function.
        # By redefining these variables as local, we save a lot of typing
        #  and possible errors.
        Thor = self.Thor
        AgentCapabilities = self.AgentCapabilities
        Tasks = self.Tasks
        CommWindowsBandwidth = self.CommWindowsBandwidth
        Options = self.Options

        if "CostFunction" in Options.keys():
            CostFunction = Options["CostFunction"]
        else:
            CostFunction = {"energy": 0.0,
                            "total_task_reward": 1.0, "total_time": 0.0}
            print("WARNING: default cost function used")
        if CostFunction["total_time"] != 0:
            print("WARNING: minimum-makespan optimization is not supported at this time.")

        TasksList = list(Tasks.OptionalTasks.keys())
        AgentsList = list(
            AgentCapabilities.ComputationLoad[TasksList[0]].keys())

        info_weight = 0.001

        # We find things
        ix = 0

        X = {}
        X_cost_optional = []
        X_cost_energy = []
        X_name = []
        for i in AgentsList:
            X[i] = {}
            for m in TasksList:
                X[i][m] = {}
                for t in range(Thor - AgentCapabilities.ComputationTime[m][i]):
                    if t >= 0:
                        X[i][m][t] = ix
                        ix += 1
                        X_cost_optional.append(
                            Tasks.TaskReward[m] * Tasks.OptionalTasks[m])
                        X_cost_energy.append(
                            -AgentCapabilities.EnergyCost[m][i])
                        X_name.append('X[{},{},{}]'.format(i, m, t))
        x_vars_len = ix

        D = {}
        D_UB = []
        D_name = []
        for i in AgentsList:
            D[i] = {}
            for m in TasksList:
                D[i][m] = {}
                for t in range(Thor):
                    D[i][m][t] = ix
                    D_name.append('D[{},{},{}]'.format(i, m, t))
                    ix += 1
                    if t == 0:
                        D_UB.append(AgentCapabilities.InitialInformation[m][i])
                    else:
                        D_UB.append(1)

        d_vars_len = ix - x_vars_len
        C = {}
        C_cost_optional = []
        C_cost_energy = []
        C_name = []
        for i in AgentsList:
            C[i] = {}
            for j in AgentsList:
                C[i][j] = {}
                for m in TasksList:
                    C[i][j][m] = {}
                    for t in range(Thor):
                        if CommWindowsBandwidth[t][i][j] > 0:
                            C[i][j][m][t] = ix
                            C_cost_optional.append(-info_weight)
                            if Options['Packet Comms'] is True:
                                temp_energy_cost = float(
                                    self.CommEnergyCost[t][i][j] * CommWindowsBandwidth[t][i][j])
                            else:
                                temp_energy_cost = 0.
                                end_comm_time = self._FindEndCommTime(
                                    i, j, m, t)
                                if end_comm_time is None:
                                    end_comm_time = Thor+1
                                for transmission_time in range(t, end_comm_time):
                                    temp_energy_cost += (
                                        self.CommEnergyCost[t][i][j] * CommWindowsBandwidth[t][i][j])
                            C_cost_energy.append(-temp_energy_cost)
                            C_name.append('C[{},{},{},{}]'.format(i, j, m, t))
                            ix += 1

        comm_vars_length = ix - d_vars_len - x_vars_len

        I = {}
        I_name = []
        for t in range(Thor):
            I[t] = ix
            I_name.append('I[{}]'.format(t))
            ix += 1
        idle_vars_length = ix - comm_vars_length - d_vars_len - x_vars_len

        X_cost = CostFunction["total_task_reward"] * np.array(
            X_cost_optional, dtype=float) + CostFunction["energy"] * np.array(X_cost_energy, dtype=float)
        C_cost = CostFunction["total_task_reward"] * np.array(
            C_cost_optional, dtype=float) + CostFunction["energy"] * np.array(C_cost_energy, dtype=float)

        # We create the variables
        self.cpx.variables.add(
            obj=X_cost,
            lb=[0] * x_vars_len,
            ub=[1] * x_vars_len,
            types=['B'] * x_vars_len,
            names=X_name,
        )

        self.cpx.variables.add(
            obj=[0] * d_vars_len,
            lb=[0] * d_vars_len,
            ub=D_UB,
            types=['B'] * d_vars_len,
            names=D_name,
        )

        self.cpx.variables.add(
            obj=C_cost,
            lb=[0] * comm_vars_length,
            ub=[1] * comm_vars_length,
            types=['B'] * comm_vars_length,
            names=C_name,
        )

        # Constraints
        self._verbprint("Constraint: Getting the job done")
        self.cpx.linear_constraints.add(lin_expr=[cplex.SparsePair([X[i][m][t] for i in AgentsList for t in range(0, max([0, Thor - AgentCapabilities.ComputationTime[m][i]]))],
                                                                   [1.0 for i in AgentsList for t in range(0, max([0, Thor - AgentCapabilities.ComputationTime[m][i]]))])
                                                  for m in TasksList if Tasks.OptionalTasks[m] is False],
                                        senses=[
                                            'G' for m in TasksList if Tasks.OptionalTasks[m] is False],
                                        rhs=[1.0 for m in TasksList if Tasks.OptionalTasks[m] is False])
        self._verbprint("Constraint: Getting the job done once")
        self.cpx.linear_constraints.add(lin_expr=[cplex.SparsePair([X[i][m][t] for i in AgentsList for t in range(0, max([0, Thor - AgentCapabilities.ComputationTime[m][i]]))],
                                                                   [1.0 for i in AgentsList for t in range(0, max([0, Thor - AgentCapabilities.ComputationTime[m][i]]))])
                                                  for m in TasksList],
                                        senses=['L' for m in TasksList],
                                        rhs=[1.0 for m in TasksList])

        self._verbprint("Constraint: Incompatible tasks")
        self.cpx.linear_constraints.add(lin_expr=[cplex.SparsePair([X[i][BadTask][t] for i in AgentsList for BadTask in BadPairing for t in range(Thor - AgentCapabilities.ComputationTime[BadTask][i]) if t >= 0],
                                                                   [1 for i in AgentsList for BadTask in BadPairing for t in range(Thor - AgentCapabilities.ComputationTime[BadTask][i]) if t >= 0])
                                                  for BadPairing in Tasks.IncompatibleTasks],
                                        senses=[
                                            'L' for BadPairing in Tasks.IncompatibleTasks],
                                        rhs=[1 for BadPairing in Tasks.IncompatibleTasks])

        self._verbprint("Constraint: Task Dependencies")
        TaskDependencyConstraintList = []
        TaskDependencyConstraintList
        for i in AgentsList:
            for m in TasksList:
                if m in set(Tasks.DependencyList.keys()):
                    for t in range(Thor - AgentCapabilities.ComputationTime[m][i]):
                        if Tasks.DependencyList[m]:
                            for DepTasks in Tasks.DependencyList[m]:
                                if DepTasks:
                                    TaskDependencyConstraintList += [cplex.SparsePair([X[i][m][t]] + [D[i][predecessor][t] for predecessor in DepTasks],
                                                                                      [1] + [-1 for predecessor in DepTasks])]
        self.cpx.linear_constraints.add(lin_expr=TaskDependencyConstraintList,
                                        senses=['L'] *
                                        len(TaskDependencyConstraintList),
                                        rhs=[0] * len(TaskDependencyConstraintList))

        if Options['Packet Comms'] is True:
            self._verbprint("Constraint: Resource availability")
            RAConstraintList = []
            for i in AgentsList:
                for t in range(Thor):
                    VarsList = [X[i][m][tt] for m in TasksList for tt in range(max(
                        0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1) if m in X[i].keys() if tt in X[i][m].keys()]
                    CoeffList = [AgentCapabilities.ComputationLoad[m][i] for m in TasksList for tt in range(max(
                        0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1) if m in X[i].keys() if tt in X[i][m].keys()]
                    VarsList += [C[i][j][m][t]
                                 for m in TasksList for j in AgentsList if CommWindowsBandwidth[t][i][j] > 0 and j != i]
                    CoeffList += [
                        1 for m in TasksList for j in AgentsList if CommWindowsBandwidth[t][i][j] > 0 and j != i]
                    VarsList += [C[j][i][m][t]
                                 for m in TasksList for j in AgentsList if CommWindowsBandwidth[t][j][i] > 0 and j != i]
                    CoeffList += [
                        1 for m in TasksList for j in AgentsList if CommWindowsBandwidth[t][j][i] > 0 and j != i]
                    RAConstraintList += [cplex.SparsePair(VarsList, CoeffList)]
            self.cpx.linear_constraints.add(lin_expr=RAConstraintList,
                                            senses=['L'] *
                                            len(RAConstraintList),
                                            rhs=[1] * len(RAConstraintList))

            self._verbprint("Constraint: New information acquisition")
            # New information acquisition
            NIConstraintList = []
            for i in AgentsList:
                for m in TasksList:
                    for t in range(Thor - 1):
                        VarsList = [D[i][m][t + 1], D[i][m][t]]
                        CoeffList = [1, -1]
                        VarsList += [C[j][i][m][tt] for j in AgentsList for tt in range(
                            t + 1) if CommWindowsBandwidth[tt][j][i] > 0]
                        CoeffList += [-np.min([1., np.double(CommWindowsBandwidth[tt][j][i]) / np.double(Tasks.ProductsSize[m])])
                                      for j in AgentsList for tt in range(t + 1) if CommWindowsBandwidth[tt][j][i] > 0]
                        if t - AgentCapabilities.ComputationTime[m][i] + 1 >= 0:
                            VarsList += [X[i][m][t -
                                                 AgentCapabilities.ComputationTime[m][i] + 1]]
                            CoeffList += [-1]
                        NIConstraintList += [
                            cplex.SparsePair(VarsList, CoeffList)]
            self.cpx.linear_constraints.add(lin_expr=NIConstraintList,
                                            senses=['L'] *
                                            len(NIConstraintList),
                                            rhs=[0] * len(NIConstraintList))

        else:
            self._ComputeCumulativeCommBandwidth()

            self._verbprint(
                "Constraint: Resource availability (no packet comms)")
            # Resource availability
            RAConstraintList = []
            for i in AgentsList:
                for t in range(Thor):
                    VarsList = [X[i][m][tt] for m in TasksList for tt in range(max(
                        0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1) if tt in X[i][m].keys()]
                    CoeffList = [AgentCapabilities.ComputationLoad[m][i] for m in TasksList for tt in range(
                        max(0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1) if tt in X[i][m].keys()]
                    for m in TasksList:
                        for j in AgentsList:
                            MCT_to = self._FindMaxStartCommTime(i, j, m, t)
                            if MCT_to is None:
                                MCT_to = - 1
                            VarsList += [C[i][j][m][tau] for tau in range(
                                MCT_to + 1, t + 1) if CommWindowsBandwidth[tau][i][j] > 0 and j != i]
                            CoeffList += [1 for tau in range(
                                MCT_to + 1, t + 1) if CommWindowsBandwidth[tau][i][j] > 0 and j != i]
                            MCT_from = self._FindMaxStartCommTime(j, i, m, t)
                            if MCT_from is None:
                                MCT_from = -1
                            VarsList += [C[j][i][m][tau] for tau in range(
                                MCT_from + 1, t + 1) if CommWindowsBandwidth[tau][j][i] > 0 and j != i]
                            CoeffList += [1 for tau in range(
                                MCT_from + 1, t + 1) if CommWindowsBandwidth[tau][j][i] > 0 and j != i]
                    RAConstraintList += [cplex.SparsePair(VarsList, CoeffList)]
            self.cpx.linear_constraints.add(lin_expr=RAConstraintList,
                                            senses=['L'] *
                                            len(RAConstraintList),
                                            rhs=[1] * len(RAConstraintList))

            self._verbprint(
                "Constraint: New information acquisition (no packet comms)")
            NIConstraintList = []
            # New information acquisition
            for i in AgentsList:
                for m in TasksList:
                    for t in range(Thor - 1):
                        VarsList = [D[i][m][t + 1], D[i][m][t]]
                        CoeffList = [1, -1]
                        for j in AgentsList:
                            # This is the maximum time so that we will have received the information by time t+1
                            MCT = self._FindMaxStartCommTime(j, i, m, t + 1)
                            if MCT is not None:
                                VarsList += [C[j][i][m][tt]
                                             for tt in range(MCT + 1) if CommWindowsBandwidth[tt][j][i] > 0]
                                CoeffList += [-1 for tt in range(
                                    MCT + 1) if CommWindowsBandwidth[tt][j][i] > 0]
                        if t - AgentCapabilities.ComputationTime[m][i] + 1 >= 0:
                            VarsList += [X[i][m][t -
                                                 AgentCapabilities.ComputationTime[m][i] + 1]]
                            CoeffList += [-1]
                        NIConstraintList += [
                            cplex.SparsePair(VarsList, CoeffList)]
            self.cpx.linear_constraints.add(lin_expr=NIConstraintList,
                                            senses=['L'] *
                                            len(NIConstraintList),
                                            rhs=[0] * len(NIConstraintList))

        self._verbprint("Constraint: Knowledge")
        self.cpx.linear_constraints.add(lin_expr=[cplex.SparsePair([C[i][j][m][t], D[i][m][t]], [1, -1]) for i in AgentsList for j in AgentsList for m in TasksList for t in range(Thor) if CommWindowsBandwidth[t][i][j] > 0],
                                        senses=['L' for i in AgentsList for j in AgentsList for m in TasksList for t in range(
                                            Thor) if CommWindowsBandwidth[t][i][j] > 0],
                                        rhs=[0 for i in AgentsList for j in AgentsList for m in TasksList for t in range(Thor) if CommWindowsBandwidth[t][i][j] > 0])

        self.SetupEndTime = time.time() - SetupStartTime
        self._verbprint('Setup time: {}'.format(self.SetupEndTime))
        self.X = X
        self.D = D
        self.C = C
        self.x_vars_len = x_vars_len
        self.d_vars_len = d_vars_len
        self.comm_vars_length = comm_vars_length
        self.TasksList = TasksList
        self.AgentsList = AgentsList
        self.problemIsSetUp = True

    def solve(self):
        if self.problemIsSetUp is False:
            self.setUp()
        SolveStartTime = time.time()
        self.cpx.parameters.mip.tolerances.mipgap.set(0.05)

        if self.Verbose is False:
            self.cpx.set_log_stream(None)
            self.cpx.set_warning_stream(None)
            self.cpx.set_results_stream(None)
        self.cpx.solve()
        self._verbprint('Solution status:                   %d' %
                        self.cpx.solution.get_status())
        if self.cpx.solution.is_primal_feasible():
            self.opt_val = self.cpx.solution.get_objective_value()
            self._verbprint(
                'Optimal value:                     {}'.format(self.opt_val))
            self.values = self.cpx.solution.get_values()
            # Solution in MIP Start format
            varNames = self.cpx.variables.get_names()
            varValues = self.cpx.solution.get_values(varNames)
            self.CPLEXSol = [varNames, varValues]
        else:
            self.opt_val = None
            self.values = np.zeros([self.cpx.variables.get_num()])
        self.SolveEndTime = time.time() - SolveStartTime
        self._verbprint('Solve time: {}'.format(self.SolveEndTime))

        # This is post-processing and should probably be moved to a different function so it can be called on an existing solution.
        TaskSchedule = {}
        TaskAgent = {}

        # This is a ugly hack. Sometimes CPLEX will return weird fractional solutions even if we require integral ones. Here, we round.
        soleps = np.finfo(np.float32).eps
        for m in self.TasksList:
            FOUND_FLAG = False
            for t in range(self.Thor):
                for i in self.AgentsList:
                    if t in self.X[i][m].keys() and self.values[self.X[i][m][t]] >= 1. - soleps:
                        TaskSchedule[m] = t
                        TaskAgent[m] = i
                        FOUND_FLAG = True
                        break
            if not FOUND_FLAG:
                TaskSchedule[m] = None
                TaskAgent[m] = None
        TaskAssignment = {'TaskSchedule': TaskSchedule,
                          'TaskAgent': TaskAgent, 'Cost': self.opt_val}

        CommSchedule = []
        for i in self.AgentsList:
            for j in self.AgentsList:
                for m in self.TasksList:
                    for t in range(self.Thor):
                        if self.CommWindowsBandwidth[t][i][j] > 0 and self.values[self.C[i][j][m][t]] > 0:
                            if self.Options['Packet Comms'] is True:
                                CommSchedule.append([i, j, m, t])
                            else:
                                EndCommTime = self._FindEndCommTime(i, j, m, t)
                                CommSchedule.append([i, j, m, t, EndCommTime])

        RawOutput = {}
        RawOutput['X'] = self.values[0:self.x_vars_len]
        RawOutput['D'] = self.values[self.x_vars_len:self.x_vars_len + self.d_vars_len]
        RawOutput['C'] = self.values[self.x_vars_len + self.d_vars_len:]
        RawOutput['Fval'] = self.opt_val
        RawOutput['Exitflag'] = self.cpx.solution.get_status()
        RawOutput['Exitstring'] = self.cpx.solution.get_status_string()
        RawOutput['Setup time'] = self.SetupEndTime
        RawOutput['Total time'] = self.SetupEndTime + self.SolveEndTime
        RawOutput['Cplex problem'] = self.cpx
        self.TaskAssignment = TaskAssignment
        self.RawOutput = RawOutput
        self.CommSchedule = CommSchedule
        self.problemIsSolved = True
        if self.cpx.solution.is_primal_feasible():
            self.problemIsFeasible = True

    def getMIPStart(self):
        if self.problemIsFeasible:
            return self.CPLEXSol
        else:
            return None

    def setMIPStart(self, MIPStart, effort_level=0):
        self.cpx.MIP_starts.add(MIPStart, effort_level)

    def schedule(self):
        # Override to implement time limit
        startSetupTime = time.time()
        if self.problemIsSetUp is False:
            self.setUp()
        endSetupTime = time.time() - startSetupTime
        self._verbprint("Setup time (s): {}".format(endSetupTime))
        # Set clock time limit
        overallTimeLimit = self.cpx.parameters.timelimit.get()
        self._verbprint("Overall time limit: {}".format(overallTimeLimit))
        solverTimeLimit = overallTimeLimit - endSetupTime
        if solverTimeLimit <= 0:
            return None
        self._verbprint(
            "Setting solver time limit: {}".format(solverTimeLimit))
        self.cpx.parameters.timelimit.set(solverTimeLimit)
        startSolveTime = time.time()
        self.solve()
        endSolveTime = time.time() - startSolveTime
        self._verbprint("Solution time: {}".format(endSolveTime))
        self._verbprint("Overall time: {}".format(endSolveTime + endSetupTime))

        if self.problemIsFeasible is False:
            print("Problem infeasible!")
            return None
        self.formatToJSON()
        return self.Timeline

    def getSolverState(self):
        if self.problemIsSolved is False:
            return "Solver not called"
        else:
            return self.cpx.solution.get_status()

    def getProblem(self):
        return self.cpx

    def writeSolution(self, filename='MIPstarts'):
        self.cpx.parameters.output.writelevel.set(4)
        self.cpx.MIP_starts.write(filename + '.mps')
        self.cpx.solution.write(filename + '.sol')

    def getOptimizationTerminator(self):
        ''' Gets an object that can be called to stop the optimization process.
            To actually terminate the problem, call the method abort() of the
            returned object.
            '''
        return self.aborter

    def setTimeLimits(self, DetTicksLimit=1e75, ClockTimeLimit=1e75):
        # Deterministic time limit
        self.cpx.parameters.dettimelimit.set(DetTicksLimit)
        # clock time limit
        self.cpx.parameters.timelimit.set(ClockTimeLimit)

    def setSolverParameters(self, Properties):
        for param in Properties:
            propset = self.cpx.parameters
            for prop in param:
                propset = getattr(propset, prop)
            propset.set(Properties[param])


class MOSAICSCIPScheduler(MOSAICMILPScheduler):
    """
    An implementation of the MILP scheduler that uses the SCIP solver.
    """

    def solverSpecificInit(self):
        if SCIPAvailable is False:
            print("SCIP not available. This will not work")
            return
        # # Create the SCIP problem
        self.cpx = scip.Model("tv-milp")
        if self.Verbose is False:
            self.cpx.hideOutput()
        self.aborter = None  # No aborter for now
        self.TimeLimit = None
        if self.Options['Packet Comms'] is not True:
            self._ComputeCumulativeCommBandwidth()

    def setUp(self):
        if self.problemIsSetUp:
            return
        SetupStartTime = time.time()

        # This used to be a part of a larger function.
        # By redefining these variables as local, we save a lot of typing
        #  and possible errors.
        Thor = self.Thor
        AgentCapabilities = self.AgentCapabilities
        Tasks = self.Tasks
        CommWindowsBandwidth = self.CommWindowsBandwidth
        Options = self.Options

        if "CostFunction" in Options.keys():
            CostFunction = Options["CostFunction"]
        else:
            CostFunction = {"energy": 0.0,
                            "total_task_reward": 1.0, "total_time": 0.0}
            print("WARNING: default cost function used")
        if CostFunction["total_time"] != 0:
            print("WARNING: minimum-makespan optimization is not supported at this time.")

        TasksList = list(Tasks.OptionalTasks.keys())
        AgentsList = list(
            AgentCapabilities.ComputationLoad[TasksList[0]].keys())

        info_weight = 0.001

        # We find things
        ix = 0
        X = {}
        X_cost = {}
        for i in AgentsList:
            X[i] = {}
            X_cost[i] = {}
            for m in TasksList:
                X[i][m] = {}
                X_cost[i][m] = {}
                for t in range(Thor - AgentCapabilities.ComputationTime[m][i]):
                    if t >= 0:
                        ix += 1
                        X[i][m][t] = self.cpx.addVar(
                            vtype='B',
                            name='X[{},{},{}]'.format(i, m, t)
                        )
                        X_cost[i][m][t] = -CostFunction["total_task_reward"]*float(
                            Tasks.TaskReward[m] * Tasks.OptionalTasks[m]) + CostFunction["energy"]*float(AgentCapabilities.EnergyCost[m][i])
        x_vars_len = ix

        D = {}
        D_cost = {}
        for i in AgentsList:
            D[i] = {}
            D_cost[i] = {}
            for m in TasksList:
                D[i][m] = {}
                D_cost[i][m] = {}
                for t in range(Thor):
                    ix += 1
                    if t == 0:
                        D_UB = AgentCapabilities.InitialInformation[m][i]
                    else:
                        D_UB = 1
                    D[i][m][t] = self.cpx.addVar(
                        vtype="I",  # If 'B' is specified, UBs and LBs are ignored
                        lb=0,
                        ub=D_UB,
                        name='D[{},{},{}]'.format(i, m, t)
                    )
                    D_cost[i][m][t] = 0.

        d_vars_len = ix - x_vars_len
        C = {}
        C_cost = {}
        for i in AgentsList:
            C[i] = {}
            C_cost[i] = {}
            for j in AgentsList:
                C[i][j] = {}
                C_cost[i][j] = {}
                for m in TasksList:
                    C[i][j][m] = {}
                    C_cost[i][j][m] = {}
                    for t in range(Thor):
                        if CommWindowsBandwidth[t][i][j] > 0:
                            C[i][j][m][t] = self.cpx.addVar(
                                vtype="B",
                                name='C[{},{},{},{}]'.format(i, j, m, t)
                            )
                            if Options['Packet Comms'] is True:
                                temp_energy_cost = float(
                                    self.CommEnergyCost[t][i][j] * CommWindowsBandwidth[t][i][j])
                            else:
                                temp_energy_cost = 0.
                                end_comm_time = self._FindEndCommTime(
                                    i, j, m, t)
                                if end_comm_time is None:
                                    end_comm_time = Thor+1
                                for transmission_time in range(t, end_comm_time):
                                    temp_energy_cost += (
                                        self.CommEnergyCost[t][i][j] * CommWindowsBandwidth[t][i][j])
                                    # TODO should t be transmission_time here?
                            C_cost[i][j][m][t] = CostFunction["total_task_reward"] * \
                                info_weight + \
                                CostFunction["energy"]*float(temp_energy_cost)
                            ix += 1

        comm_vars_length = ix - d_vars_len - x_vars_len

        # Set the objective
        self.cpx.setObjective(
            scip.quicksum(X_cost[i][m][t]*X[i][m][t]
                          for i in AgentsList
                          for m in TasksList
                          for t in range(Thor - AgentCapabilities.ComputationTime[m][i])
                          )
            + scip.quicksum(C_cost[i][j][m][t]*C[i][j][m][t]
                            for i in AgentsList
                            for j in AgentsList
                            for m in TasksList
                            for t in range(Thor)
                            if CommWindowsBandwidth[t][i][j] > 0
                            ),
            "minimize"
        )

        # Constraints
        self._verbprint("Constraint: Getting the job done")
        for m in TasksList:
            if Tasks.OptionalTasks[m] is False:
                self.cpx.addCons(scip.quicksum(X[i][m][t] for i in AgentsList for t in range(
                    0, max([0, Thor - AgentCapabilities.ComputationTime[m][i]]))) >= 1.)

        self._verbprint("Constraint: Getting the job done once")
        for m in TasksList:
            self.cpx.addCons(scip.quicksum(X[i][m][t] for i in AgentsList for t in range(
                0, max([0, Thor - AgentCapabilities.ComputationTime[m][i]]))) <= 1.)

        self._verbprint("Constraint: Incompatible tasks")
        for BadPairing in Tasks.IncompatibleTasks:
            self.cpx.addCons(scip.quicksum(X[i][BadTask][t] for i in AgentsList for BadTask in BadPairing for t in range(Thor - AgentCapabilities.ComputationTime[BadTask][i]) if t >= 0)
                             <= 1.)

        self._verbprint("Constraint: Task Dependencies")
        for i in AgentsList:
            for m in TasksList:
                if m in set(Tasks.DependencyList.keys()):
                    for t in range(Thor - AgentCapabilities.ComputationTime[m][i]):
                        if Tasks.DependencyList[m]:
                            for DepTasks in Tasks.DependencyList[m]:
                                if DepTasks:
                                    self.cpx.addCons(
                                        X[i][m][t] - scip.quicksum(D[i][predecessor][t]
                                                                   for predecessor in DepTasks) <= 0
                                    )

        if Options['Packet Comms'] is True:
            self._verbprint("Constraint: Resource availability")
            for i in AgentsList:
                for t in range(Thor):
                    self.cpx.addCons(
                        scip.quicksum(AgentCapabilities.ComputationLoad[m][i]*X[i][m][tt]
                                      for m in TasksList for tt in range(max(0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1) if m in X[i].keys() if tt in X[i][m].keys())
                        + scip.quicksum(C[i][j][m][t]
                                        for m in TasksList for j in AgentsList if CommWindowsBandwidth[t][i][j] > 0 and j != i)
                        + scip.quicksum(C[j][i][m][t]
                                        for m in TasksList for j in AgentsList if CommWindowsBandwidth[t][j][i] > 0 and j != i)
                        <= 1
                    )

            self._verbprint("Constraint: New information acquisition")
            # New information acquisition
            for i in AgentsList:
                for m in TasksList:
                    for t in range(Thor - 1):
                        if t - AgentCapabilities.ComputationTime[m][i] + 1 >= 0:
                            self.cpx.addCons(
                                D[i][m][t + 1] - D[i][m][t] +
                                scip.quicksum(-np.min([1., np.double(CommWindowsBandwidth[tt][j][i]) / np.double(Tasks.ProductsSize[m])])*C[j][i][m][tt] for j in AgentsList for tt in range(t + 1) if CommWindowsBandwidth[tt][j][i] > 0) +
                                - X[i][m][t -
                                          AgentCapabilities.ComputationTime[m][i] + 1]
                                <= 0
                            )
                        else:
                            self.cpx.addCons(
                                D[i][m][t + 1] - D[i][m][t] +
                                scip.quicksum(-np.min([1., np.double(CommWindowsBandwidth[tt][j][i]) / np.double(Tasks.ProductsSize[m])])
                                              * C[j][i][m][tt] for j in AgentsList for tt in range(t + 1) if CommWindowsBandwidth[tt][j][i] > 0)
                                <= 0
                            )

        else:
            self._ComputeCumulativeCommBandwidth()

            self._verbprint(
                "Constraint: Resource availability (no packet comms)")
            # Resource availability
            for i in AgentsList:
                for t in range(Thor):
                    local_processes = scip.quicksum(AgentCapabilities.ComputationLoad[m][i]*X[i][m][tt]
                                                    for m in TasksList for tt in range(max(0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1) if tt in X[i][m].keys())

                    for m in TasksList:
                        for j in AgentsList:
                            MCT_to = self._FindMaxStartCommTime(i, j, m, t)
                            if MCT_to is None:
                                MCT_to = - 1
                            outbound_comms = scip.quicksum(C[i][j][m][tau] for tau in range(
                                MCT_to + 1, t + 1) if CommWindowsBandwidth[tau][i][j] > 0 and j != i)
                            MCT_from = self._FindMaxStartCommTime(j, i, m, t)
                            if MCT_from is None:
                                MCT_from = -1
                            inbound_comms = scip.quicksum(C[j][i][m][tau] for tau in range(
                                MCT_from + 1, t + 1) if CommWindowsBandwidth[tau][j][i] > 0 and j != i)
                    self.cpx.addCons(
                        local_processes+outbound_comms+inbound_comms <= 1
                    )

            self._verbprint(
                "Constraint: New information acquisition (no packet comms)")
            # New information acquisition
            for i in AgentsList:
                for m in TasksList:
                    for t in range(Thor - 1):
                        VarsListInboundInfo = []
                        for j in AgentsList:
                            # This is the maximum time so that we will have received the information by time t+1
                            MCT = self._FindMaxStartCommTime(j, i, m, t + 1)
                            if MCT is not None:
                                VarsListInboundInfo += [C[j][i][m][tt] for tt in range(
                                    MCT + 1) if CommWindowsBandwidth[tt][j][i] > 0]
                        if t - AgentCapabilities.ComputationTime[m][i] + 1 >= 0:
                            VarsListInboundInfo += [X[i][m][t -
                                                            AgentCapabilities.ComputationTime[m][i] + 1]]

                        self.cpx.addCons(
                            D[i][m][t + 1] - D[i][m][t]
                            - scip.quicksum(VarsListInboundInfo)
                            <= 0.
                        )

        self._verbprint("Constraint: Knowledge")
        for i in AgentsList:
            for j in AgentsList:
                for m in TasksList:
                    for t in range(Thor):
                        if CommWindowsBandwidth[t][i][j] > 0:
                            self.cpx.addCons(
                                C[i][j][m][t] - D[i][m][t] <= 0
                            )

        self.SetupEndTime = time.time() - SetupStartTime
        self._verbprint('Setup time: {}'.format(self.SetupEndTime))
        self.X = X
        self.D = D
        self.C = C
        self.x_vars_len = x_vars_len
        self.d_vars_len = d_vars_len
        self.comm_vars_length = comm_vars_length
        self.TasksList = TasksList
        self.AgentsList = AgentsList
        self.problemIsSetUp = True

    def solve(self):
        if self.problemIsSetUp is False:
            self.setUp()
        SolveStartTime = time.time()
        self.cpx.setRealParam('limits/gap', 0.05)
        if self.TimeLimit is not None:
            self.cpx.setRealParam('limits/time', self.TimeLimit)

        self.cpx.optimize()
        self._verbprint(
            'Solution status:                   {}'.format(self.cpx.getStatus()))
        if self.cpx.getStatus() != 'infeasible' and self.cpx.getStatus() != 'unbounded':
            self.opt_val = self.cpx.getObjVal()
            self._verbprint(
                'Optimal value:                     {}'.format(self.opt_val))
        else:
            self.opt_val = None
        self.SolveEndTime = time.time() - SolveStartTime
        self._verbprint('Solve time: {}'.format(self.SolveEndTime))

        # This is post-processing and should probably be moved to a different function so it can be called on an existing solution.
        TaskSchedule = {}
        TaskAgent = {}

        # This is a ugly hack. Sometimes the solver will return weird fractional solutions even if we require integral ones. Here, we round.
        soleps = np.finfo(np.float32).eps
        for m in self.TasksList:
            FOUND_FLAG = False
            for t in range(self.Thor):
                for i in self.AgentsList:
                    if t in self.X[i][m].keys() and self.cpx.getVal(self.X[i][m][t]) >= 1. - soleps:
                        TaskSchedule[m] = t
                        TaskAgent[m] = i
                        FOUND_FLAG = True
            if not FOUND_FLAG:
                TaskSchedule[m] = None
                TaskAgent[m] = None
        TaskAssignment = {'TaskSchedule': TaskSchedule,
                          'TaskAgent': TaskAgent, 'Cost': self.opt_val}

        CommSchedule = []
        for i in self.AgentsList:
            for j in self.AgentsList:
                for m in self.TasksList:
                    for t in range(self.Thor):
                        if self.CommWindowsBandwidth[t][i][j] > 0 and self.cpx.getVal(self.C[i][j][m][t]) > 0:
                            if self.Options['Packet Comms'] is True:
                                CommSchedule.append([i, j, m, t])
                            else:
                                EndCommTime = self._FindEndCommTime(i, j, m, t)
                                CommSchedule.append([i, j, m, t, EndCommTime])

        RawOutput = {}
        RawOutput['X'] = self.X
        RawOutput['D'] = self.D
        RawOutput['C'] = self.C
        RawOutput['Fval'] = self.opt_val
        RawOutput['Exitflag'] = self.cpx.getStatus()
        RawOutput['Exitstring'] = self.cpx.getStatus()
        RawOutput['Setup time'] = self.SetupEndTime
        RawOutput['Total time'] = self.SetupEndTime + self.SolveEndTime
        RawOutput['Cplex problem'] = self.cpx
        self.TaskAssignment = TaskAssignment
        self.RawOutput = RawOutput
        self.CommSchedule = CommSchedule
        self.problemIsSolved = True
        if self.cpx.getStatus() != 'infeasible' and self.cpx.getStatus() != 'unbounded':
            self.problemIsFeasible = True

    def setMIPStart(self, MIPStart):
        ''' Certain solvers allow the MIP to be "seeded" with a MIP start. '''
        # See https://github.com/SCIP-Interfaces/PySCIPOpt/issues/115 for syntax.
        sol = self.cpx.createSol()
        # MIPStart is a dict containing the name of each variable and the corresponding value
        for key, val in MIPStart:
            self.cpx.setSolVal(sol, eval(key), val)
        self.cpx.addSol(sol)

    def schedule(self):
        # Override to implement time limit
        startSetupTime = time.time()
        if self.problemIsSetUp is False:
            self.setUp()
        endSetupTime = time.time() - startSetupTime
        self._verbprint("Setup time (s): {}".format(endSetupTime))
        # Set clock time limit
        if self.TimeLimit is not None:
            overallTimeLimit = self.TimeLimit
            self._verbprint("Overall time limit: {}".format(overallTimeLimit))
            solverTimeLimit = overallTimeLimit - endSetupTime
            if solverTimeLimit <= 0:
                return None
            self._verbprint(
                "Setting solver time limit: {}".format(solverTimeLimit))
            self.cpx.setRealParam('limits/time', solverTimeLimit)
        startSolveTime = time.time()
        self.solve()
        endSolveTime = time.time() - startSolveTime
        self._verbprint("Solution time: {}".format(endSolveTime))
        self._verbprint("Overall time: {}".format(endSolveTime + endSetupTime))

        if self.problemIsFeasible is False:
            print("Problem infeasible!")
            return None
        self.formatToJSON()
        return self.Timeline

    def getSolverState(self):
        if self.problemIsSolved is False:
            return "Solver not called"
        else:
            return self.cpx.getStatus()

    def getProblem(self):
        return self.cpx

    def setTimeLimits(self, DetTicksLimit=None, ClockTimeLimit=None):
        if DetTicksLimit is not None:
            print("WARNING: deterministic ticks limit ignored by SCIP")
        if ClockTimeLimit is not None:
            self.TimeLimit = ClockTimeLimit

    def setSolverParameters(self, Properties):
        for param in Properties:
            self.cpx.setRealParam(param, Properties[param])


class MOSAICPULPScheduler(MOSAICMILPScheduler):
    """
    An implementation of the MILP scheduler that uses the PuLP modeling package.
    """

    def solverSpecificInit(self):
        if self.Options['Packet Comms'] is not True:
            self._ComputeCumulativeCommBandwidth()

    def setUp(self):
        SetupStartTime = time.clock()
        self._verbprint("Building problem")

        Thor = self.Thor
        AgentCapabilities = self.AgentCapabilities
        Tasks = self.Tasks
        CommWindowsBandwidth = self.CommWindowsBandwidth
        Options = self.Options

        if "CostFunction" in Options.keys():
            CostFunction = Options["CostFunction"]
        else:
            CostFunction = {"energy": 0.0,
                            "total_task_reward": 1.0, "total_time": 0.0}
            print("WARNING: default cost function used")
        if CostFunction["total_time"] != 0:
            print("WARNING: minimum-makespan optimization is not supported at this time.")

        TasksList = list(Tasks.OptionalTasks.keys())
        AgentsList = list(
            AgentCapabilities.ComputationLoad[TasksList[0]].keys())

        self._verbprint("Creating variables")
        X = pulp.LpVariable.dicts("X", (AgentsList, TasksList, list(range(Thor))),
                                  0, 1, pulp.LpBinary)
        D = pulp.LpVariable.dicts("D", (AgentsList, TasksList, list(range(Thor))),
                                  0, 1, pulp.LpBinary)
        C = pulp.LpVariable.dicts("C", (AgentsList, AgentsList, TasksList,
                                        list(range(Thor))), 0, 1, pulp.LpBinary)

        # Create the problem
        self._verbprint("Creating problem")
        MILPproblem = pulp.LpProblem("MOSAIC MILP", pulp.LpMinimize)

        # Cost
        self._verbprint("Creating Cost: task cost")
        OptionalTaskCost = - pulp.lpSum([Tasks.TaskReward[m] * Tasks.OptionalTasks[m] * X[i][m][t] for i in AgentsList for m in TasksList
                                         for t in range(Thor - AgentCapabilities.ComputationTime[m][i])
                                         if t >= 0])
        info_weight = 0.001
        self._verbprint("Information cost")
        InfoDiffusionCost = pulp.lpSum(
            info_weight * C[i][j][m][t] for i in AgentsList for m in TasksList for t in range(Thor) for j in AgentsList)
        # TODO
        self._verbprint("Energy cost")
        EnergyCostTask = pulp.lpSum([AgentCapabilities.EnergyCost[m][i] * X[i][m][t] for i in AgentsList for m in TasksList
                                     for t in range(Thor - AgentCapabilities.ComputationTime[m][i])
                                     if t >= 0])

        temp_energy_cost = {}
        for i in AgentsList:
            temp_energy_cost[i] = {}
            for j in AgentsList:
                temp_energy_cost[i][j] = {}
                for m in TasksList:
                    temp_energy_cost[i][j][m] = {}
                    for t in range(Thor):
                        if CommWindowsBandwidth[t][i][j] > 0:
                            if Options['Packet Comms'] is True:
                                temp_energy_cost[i][j][m][t] = float(
                                    self.CommEnergyCost[t][i][j] * CommWindowsBandwidth[t][i][j])
                            else:
                                temp_energy_cost[i][j][m][t] = 0.
                                end_comm_time = self._FindEndCommTime(
                                    i, j, m, t)
                                if end_comm_time is None:
                                    end_comm_time = Thor+1
                                for transmission_time in range(t, end_comm_time):
                                    temp_energy_cost[i][j][m][t] += (
                                        self.CommEnergyCost[t][i][j] * CommWindowsBandwidth[t][i][j])

        EnergyCostComm = pulp.lpSum(temp_energy_cost[i][j][m][t] * C[i][j][m][t]
                                    for i in AgentsList for m in TasksList for t in range(Thor) for j in AgentsList if CommWindowsBandwidth[t][i][j] > 0)
        # /TODO

        # Note the [0], because Numpy is deeply weird
        MILPproblem += CostFunction["total_task_reward"] * (
            OptionalTaskCost + InfoDiffusionCost) + CostFunction["energy"] * (EnergyCostTask+EnergyCostComm)

        self._verbprint("Constraint: Getting the job done")
        # Constraints
        # Getting the job done
        for m in TasksList:
            if Tasks.OptionalTasks[m] is False:
                MILPproblem += pulp.lpSum(X[i][m][t] for i in AgentsList for t in range(0, max(
                    [0, Thor - AgentCapabilities.ComputationTime[m][i]]))) >= 1, "Job {} done".format(m)
            MILPproblem += pulp.lpSum(X[i][m][t] for i in AgentsList for t in range(
                Thor)) <= 1, "Job {} done once".format(m)

        self._verbprint("Constraint: Incompatible tasks")
        # Incompatible tasks
        IncompatibilityCounter = 0
        for BadPairing in Tasks.IncompatibleTasks:
            MILPproblem += pulp.lpSum(X[i][BadTask][t] for i in AgentsList for BadTask in BadPairing for t in range(
                Thor - AgentCapabilities.ComputationTime[BadTask][i]) if t >= 0) <= 1, 'Incompatibility %d' % IncompatibilityCounter
            IncompatibilityCounter += 1

        self._verbprint("Constraint: Task Dependencies")
        # Task Dependencies
        for i in AgentsList:
            for m in TasksList:
                if m in set(Tasks.DependencyList.keys()):
                    for t in range(Thor):
                        if Tasks.DependencyList[m]:
                            DepCounter = 0
                            for DepTasks in Tasks.DependencyList[m]:
                                if DepTasks:
                                    MILPproblem += X[i][m][t] <= pulp.lpSum(
                                        D[i][predecessor][t] for predecessor in DepTasks), "Dependency {0} of task {1} for agent {2} at time {3}".format(DepCounter, m, i, t)
                                    DepCounter += 1

        if Options['Packet Comms'] is True:
            # The resource availability and information acquisition constraints
            #  are modified if communication is not packetized
            self._verbprint("Constraint: Resource availability")
            # Resource availability
            for i in AgentsList:
                for t in range(Thor):
                    MILPproblem += (pulp.lpSum(AgentCapabilities.ComputationLoad[m][i] * X[i][m][tt] for m in TasksList for tt in range(max(0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1))
                                    + pulp.lpSum(C[i][j][m][t] + C[j][i][m][t] for m in TasksList for j in AgentsList) <= 1), "Resources {0} at {1}".format(i, t)

            self._verbprint("Constraint: New information acquisition")
            # New information acquisition
            for i in AgentsList:
                for m in TasksList:
                    for t in range(Thor - 1):
                        InfoConstraint = D[i][m][t + 1] - D[i][m][t] - pulp.lpSum(C[j][i][m][tt] * np.min([1., np.double(
                            CommWindowsBandwidth[tt][j][i]) / np.double(Tasks.ProductsSize[m])]) for j in AgentsList for tt in range(t + 1)) <= 0
                        if t - AgentCapabilities.ComputationTime[m][i] + 1 >= 0:
                            InfoConstraint -= X[i][m][t -
                                                      AgentCapabilities.ComputationTime[m][i] + 1]
                        MILPproblem += InfoConstraint, "Information of task {0} at agent {1}, time {2}".format(
                            m, i, t)
        else:
            self._ComputeCumulativeCommBandwidth()

            self._verbprint(
                "Constraint: Resource availability (no packet comms)")
            # Resource availability
            for i in AgentsList:
                for t in range(Thor):
                    ResourceAvailabilityConstraintLST = [AgentCapabilities.ComputationLoad[m][i] * X[i][m][tt]
                                                         for m in TasksList for tt in range(max(0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1)]
                    for m in TasksList:
                        for j in AgentsList:
                            # If we started sending at MCT_to, we will be done just about now. So if we start at MCT_to+1, we're still going
                            MCT_to = self._FindMaxStartCommTime(i, j, m, t)
                            if MCT_to is None:
                                MCT_to = -1
                            ResourceAvailabilityConstraintLST.append(
                                [C[i][j][m][tau] for tau in range(MCT_to + 1, t + 1)])
                            MCT_from = self._FindMaxStartCommTime(j, i, m, t)
                            if MCT_from is None:
                                MCT_from = -1
                            ResourceAvailabilityConstraintLST.append(
                                [C[j][i][m][tau] for tau in range(MCT_from + 1, t + 1)])
                    ResourceAvailabilityConstraint = pulp.lpSum(
                        ResourceAvailabilityConstraintLST) <= 1
                    MILPproblem += ResourceAvailabilityConstraint, "Resources {0} at {1}".format(
                        i, t)

            self._verbprint(
                "Constraint: New information acquisition (no packet comms)")
            # New information acquisition
            for i in AgentsList:
                for m in TasksList:
                    for t in range(Thor - 1):
                        InfoConstraintLST = [D[i][m][t + 1] - D[i][m][t]]
                        for j in AgentsList:
                            # This is the maximum time so that we will have received the information by time t+1
                            MCT = self._FindMaxStartCommTime(j, i, m, t + 1)
                            if MCT is not None:
                                InfoConstraintLST.append(-pulp.lpSum(
                                    C[j][i][m][tt] for tt in range(MCT + 1)))
                        if t - AgentCapabilities.ComputationTime[m][i] + 1 >= 0:
                            InfoConstraintLST.append(
                                -X[i][m][t - AgentCapabilities.ComputationTime[m][i] + 1])
                        InfoConstraint = pulp.lpSum(InfoConstraintLST) <= 0
                        MILPproblem += InfoConstraint, "Information of task {0} at agent {1}, time {2}".format(
                            m, i, t)

        self._verbprint("Constraint: Knowledge")
        # Knowledge
        for i in AgentsList:
            for j in AgentsList:
                for m in TasksList:
                    for t in range(Thor):
                        MILPproblem += (C[i][j][m][t] <= D[i][m][t]
                                        ), "Knowledge of task {0} from ag{1} to ag{2} at t{3}".format(m, i, j, t)

        self._verbprint("Constraint: Communication Windows")
        # Communication Windows
        for i in AgentsList:
            for j in AgentsList:
                for m in TasksList:
                    for t in range(Thor):
                        CommWindowAvailable = int(
                            CommWindowsBandwidth[t][i][j] > 0)
                        C[i][j][m][t].upBound = CommWindowAvailable

        self._verbprint("Constraint: Initial Information")
        # Initial Information
        for i in AgentsList:
            for m in TasksList:
                D[i][m][0].upBound = AgentCapabilities.InitialInformation[m][i]

        SetupEndTime = time.clock()
        self.SetupTime = SetupEndTime - SetupStartTime
        self._verbprint("Setup time: " + str(self.SetupTime) + " s")
        self.MILPproblem = MILPproblem
        self.TasksList = TasksList
        self.AgentsList = AgentsList
        self.X = X
        self.D = D
        self.C = C
        self.problemIsSetUp = True

    def solve(self):
        if self.Verbose is False:
            # Redirect output to prevent PuLP from dumping data all over stdout.
            # From https://stackoverflow.com/questions/8447185/
            sys.stdout = open(os.devnull, "w")
        if self.problemIsSetUp is False:
            self.setUp()
        self._verbprint("Calling solver")
        if pulp.COIN().available() is not False:
            self._verbprint("Using COIN")
            self.MILPproblem.solve(pulp.COIN())
        elif pulp.GUROBI().available() is not False:
            self._verbprint("Using GUROBI")
            self.MILPproblem.solve(pulp.GUROBI())
        elif pulp.CPLEX().available() is not False:
            self._verbprint("Using CPLEX")
            self.MILPproblem.solve(pulp.CPLEX())
        else:
            self._verbprint("Using default solver")
            self.MILPproblem.solve()
        self._verbprint("Status: " + pulp.LpStatus[self.MILPproblem.status])

        # Unpack the output
        TaskSchedule = {}
        TaskAgent = {}
        CommSchedule = []

        if pulp.LpStatus[self.MILPproblem.status] == 'Infeasible':
            for m in self.TasksList:
                TaskSchedule[m] = None
                TaskAgent[m] = None
            CommSchedule = []
        else:
            # This is a ugly hack. Sometimes the solver will return weird fractional solutions even if we require integral ones. Here, we round.
            soleps = np.finfo(np.float32).eps
            for m in self.TasksList:
                FOUND_FLAG = False
                for t in range(self.Thor):
                    for i in self.AgentsList:
                        if self.X[i][m][t].varValue >= 1. - soleps:
                            TaskSchedule[m] = t
                            TaskAgent[m] = i
                            FOUND_FLAG = True
                            break
                if not FOUND_FLAG:
                    TaskSchedule[m] = None
                    TaskAgent[m] = None
        TaskAssignment = {}
        TaskAssignment['TaskSchedule'] = TaskSchedule
        TaskAssignment['TaskAgent'] = TaskAgent
        TaskAssignment['Cost'] = pulp.value(self.MILPproblem.objective)

        for i in self.AgentsList:
            for j in self.AgentsList:
                for m in self.TasksList:
                    for t in range(self.Thor):
                        if self.C[i][j][m][t].varValue:
                            if self.Options['Packet Comms'] is True:
                                CommSchedule.append([i, j, m, t])
                            else:
                                EndCommTime = self._FindEndCommTime(i, j, m, t)
                                CommSchedule.append([i, j, m, t, EndCommTime])

        RawOutput = {}
        RawOutput['X'] = self.X
        RawOutput['D'] = self.D
        RawOutput['C'] = self.C
        RawOutput['Fval'] = pulp.value(self.MILPproblem.objective)
        RawOutput['Exitflag'] = self.MILPproblem.status
        RawOutput['Exitstring'] = pulp.LpStatus[self.MILPproblem.status]
        RawOutput['Setup time'] = self.SetupTime
        self.TaskAssignment = TaskAssignment
        self.RawOutput = RawOutput
        self.CommSchedule = CommSchedule
        self.problemIsSolved = True
        self.problemIsFeasible = (self.MILPproblem.status > 0)
        if self.Verbose is False:
            # Restore stdout
            sys.stdout = sys.__stdout__

    def getSolverState(self):
        return pulp.LpStatus[self.MILPproblem.status]

    def getProblem(self):
        return self.MILPproblem


class MOSAICGLPKScheduler(MOSAICMILPScheduler):
    """
    An implementation of the MILP scheduler that uses the GLPK solver.
    """

    def solverSpecificInit(self):
        if GLPKAvailable is False:
            print("GLPK not available. This will not work")
            return
        # Create the GLPK problem
        self.cpx = glpk.LPX()
        #self.cpx.kind = int
        self.cpx.obj.maximize = True
        self.aborter = None  # No aborter for now
        self.TimeLimit = 1e75
        if self.Options['Packet Comms'] is not True:
            self._ComputeCumulativeCommBandwidth()

    def setUp(self):
        if self.problemIsSetUp:
            return
        SetupStartTime = time.time()

        # This used to be a part of a larger function.
        # By redefining these variables as local, we save a lot of typing
        #  and possible errors.
        Thor = self.Thor
        AgentCapabilities = self.AgentCapabilities
        Tasks = self.Tasks
        CommWindowsBandwidth = self.CommWindowsBandwidth
        Options = self.Options

        if "CostFunction" in Options.keys():
            CostFunction = Options["CostFunction"]
        else:
            CostFunction = {"energy": 0.0,
                            "total_task_reward": 1.0, "total_time": 0.0}
            print("WARNING: default cost function used")
        if CostFunction["total_time"] != 0:
            print("WARNING: minimum-makespan optimization is not supported at this time.")

        TasksList = list(Tasks.OptionalTasks.keys())
        AgentsList = list(
            AgentCapabilities.ComputationLoad[TasksList[0]].keys())

        info_weight = 0.001

        # We find things
        ix = 0

        X = {}
        X_cost_optional = []
        X_cost_energy = []
        X_name = []
        for i in AgentsList:
            X[i] = {}
            for m in TasksList:
                X[i][m] = {}
                for t in range(Thor - AgentCapabilities.ComputationTime[m][i]):
                    if t >= 0:
                        X[i][m][t] = ix
                        ix += 1
                        X_cost_optional.append(
                            Tasks.TaskReward[m] * Tasks.OptionalTasks[m])
                        X_cost_energy.append(
                            -AgentCapabilities.EnergyCost[m][i])
                        X_name.append('X[{},{},{}]'.format(i, m, t))
        x_vars_len = ix

        D = {}
        D_UB = []
        D_name = []
        for i in AgentsList:
            D[i] = {}
            for m in TasksList:
                D[i][m] = {}
                for t in range(Thor):
                    D[i][m][t] = ix
                    D_name.append('D[{},{},{}]'.format(i, m, t))
                    ix += 1
                    if t == 0:
                        D_UB.append(
                            int(AgentCapabilities.InitialInformation[m][i]))
                    else:
                        D_UB.append(1)

        d_vars_len = ix - x_vars_len
        C = {}
        C_cost_optional = []
        C_cost_energy = []
        C_name = []
        for i in AgentsList:
            C[i] = {}
            for j in AgentsList:
                C[i][j] = {}
                for m in TasksList:
                    C[i][j][m] = {}
                    for t in range(Thor):
                        if CommWindowsBandwidth[t][i][j] > 0:
                            C[i][j][m][t] = ix
                            C_cost_optional.append(-info_weight)

                            if Options['Packet Comms'] is True:
                                temp_energy_cost = float(
                                    self.CommEnergyCost[t][i][j] * CommWindowsBandwidth[t][i][j])
                            else:
                                temp_energy_cost = 0.
                                end_comm_time = self._FindEndCommTime(
                                    i, j, m, t)
                                if end_comm_time is None:
                                    end_comm_time = Thor+1
                                for transmission_time in range(t, end_comm_time):
                                    temp_energy_cost += (
                                        self.CommEnergyCost[t][i][j] * CommWindowsBandwidth[t][i][j])

                            C_cost_energy.append(-temp_energy_cost)
                            C_name.append('C[{},{},{},{}]'.format(i, j, m, t))
                            ix += 1

        comm_vars_length = ix - d_vars_len - x_vars_len

        X_cost = CostFunction["total_task_reward"] * np.array(
            X_cost_optional, dtype=float) + CostFunction["energy"] * np.array(X_cost_energy, dtype=float)
        C_cost = CostFunction["total_task_reward"] * np.array(
            C_cost_optional, dtype=float) + CostFunction["energy"] * np.array(C_cost_energy, dtype=float)

        # We create the variables
        self.cpx.cols.add(len(X_cost))
        for col_ix in range(len(X_cost)):
            c = self.cpx.cols[col_ix]
            c.kind = bool
            c.name = X_name[col_ix]
            self.cpx.obj[col_ix] = X_cost[col_ix]

        if len(D_name):
            self.cpx.cols.add(len(D_name))
            for col_ix in range(len(D_name)):
                c = self.cpx.cols[col_ix+len(X_cost)]
                c.kind = bool
                c.bounds = 0., float(D_UB[col_ix])
                c.name = D_name[col_ix]
                self.cpx.obj[col_ix+len(X_cost)] = 0.

        if len(C_name):
            self.cpx.cols.add(len(C_name))
            for col_ix in range(len(C_name)):
                c = self.cpx.cols[col_ix+len(X_cost)+len(D_name)]
                # c.bounds = 0., 1.
                c.kind = bool
                c.name = C_name[col_ix]
                self.cpx.obj[col_ix+len(X_cost)+len(D_name)] = C_cost[col_ix]

        # Constraints
        self._verbprint("Constraint: Getting the job done")
        for m in TasksList:
            if Tasks.OptionalTasks[m] is False:
                self.cpx.rows.add(1)
                self.cpx.rows[-1].matrix = [(X[i][m][t], 1.) for i in AgentsList for t in range(
                    0, max([0, Thor - AgentCapabilities.ComputationTime[m][i]]))]
                self.cpx.rows[-1].bounds = 1, None
        self._verbprint("Constraint: Getting the job done once")
        for m in TasksList:
            self.cpx.rows.add(1)
            self.cpx.rows[-1].matrix = [(X[i][m][t], 1.) for i in AgentsList for t in range(
                0, max([0, Thor - AgentCapabilities.ComputationTime[m][i]]))]
            self.cpx.rows[-1].bounds = None, 1

        self._verbprint("Constraint: Incompatible tasks")
        for BadPairing in Tasks.IncompatibleTasks:
            self.cpx.rows.add(1)
            self.cpx.rows[-1].matrix = [(X[i][BadTask][t], 1.) for i in AgentsList for BadTask in BadPairing for t in range(
                Thor - AgentCapabilities.ComputationTime[BadTask][i]) if t >= 0]
            self.cpx.rows[-1].bounds = None, 1.

        self._verbprint("Constraint: Task Dependencies")
        TaskDependencyConstraintList = []
        TaskDependencyConstraintList
        for i in AgentsList:
            for m in TasksList:
                if m in set(Tasks.DependencyList.keys()):
                    for t in range(Thor - AgentCapabilities.ComputationTime[m][i]):
                        if Tasks.DependencyList[m]:
                            for DepTasks in Tasks.DependencyList[m]:
                                if DepTasks:
                                    self.cpx.rows.add(1)
                                    self.cpx.rows[-1].matrix = [(X[i][m][t], 1)] + [(
                                        D[i][predecessor][t], -1) for predecessor in DepTasks]
                                    self.cpx.rows[-1].bounds = None, 0.

        if Options['Packet Comms'] is True:
            self._verbprint("Constraint: Resource availability")
            for i in AgentsList:
                for t in range(Thor):
                    VarsList = [X[i][m][tt] for m in TasksList for tt in range(max(
                        0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1) if m in X[i].keys() if tt in X[i][m].keys()]
                    CoeffList = [AgentCapabilities.ComputationLoad[m][i] for m in TasksList for tt in range(max(
                        0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1) if m in X[i].keys() if tt in X[i][m].keys()]
                    VarsList += [C[i][j][m][t]
                                 for m in TasksList for j in AgentsList if CommWindowsBandwidth[t][i][j] > 0 and j != i]
                    CoeffList += [
                        1 for m in TasksList for j in AgentsList if CommWindowsBandwidth[t][i][j] > 0 and j != i]
                    VarsList += [C[j][i][m][t]
                                 for m in TasksList for j in AgentsList if CommWindowsBandwidth[t][j][i] > 0 and j != i]
                    CoeffList += [
                        1 for m in TasksList for j in AgentsList if CommWindowsBandwidth[t][j][i] > 0 and j != i]
                    self.cpx.rows.add(1)
                    self.cpx.rows[-1].matrix = zip(VarsList, CoeffList)
                    self.cpx.rows[-1].bounds = None, 1

            self._verbprint("Constraint: New information acquisition")
            for i in AgentsList:
                for m in TasksList:
                    for t in range(Thor - 1):
                        VarsList = [D[i][m][t + 1], D[i][m][t]]
                        CoeffList = [1, -1]
                        VarsList += [C[j][i][m][tt] for j in AgentsList for tt in range(
                            t + 1) if CommWindowsBandwidth[tt][j][i] > 0]
                        CoeffList += [-np.min([1., np.double(CommWindowsBandwidth[tt][j][i]) / np.double(Tasks.ProductsSize[m])])
                                      for j in AgentsList for tt in range(t + 1) if CommWindowsBandwidth[tt][j][i] > 0]
                        if t - AgentCapabilities.ComputationTime[m][i] + 1 >= 0:
                            VarsList += [X[i][m][t -
                                                 AgentCapabilities.ComputationTime[m][i] + 1]]
                            CoeffList += [-1]
                        self.cpx.rows.add(1)
                        self.cpx.rows[-1].matrix = zip(VarsList, CoeffList)
                        self.cpx.rows[-1].bounds = None, 0.

        else:
            self._ComputeCumulativeCommBandwidth()

            self._verbprint(
                "Constraint: Resource availability (no packet comms)")
            for i in AgentsList:
                for t in range(Thor):
                    VarsList = [X[i][m][tt] for m in TasksList for tt in range(max(
                        0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1) if tt in X[i][m].keys()]
                    CoeffList = [AgentCapabilities.ComputationLoad[m][i] for m in TasksList for tt in range(
                        max(0, t - AgentCapabilities.ComputationTime[m][i] + 1), t + 1) if tt in X[i][m].keys()]
                    for m in TasksList:
                        for j in AgentsList:
                            MCT_to = self._FindMaxStartCommTime(i, j, m, t)
                            if MCT_to is None:
                                MCT_to = - 1
                            VarsList += [C[i][j][m][tau] for tau in range(
                                MCT_to + 1, t + 1) if CommWindowsBandwidth[tau][i][j] > 0 and j != i]
                            CoeffList += [1 for tau in range(
                                MCT_to + 1, t + 1) if CommWindowsBandwidth[tau][i][j] > 0 and j != i]
                            MCT_from = self._FindMaxStartCommTime(j, i, m, t)
                            if MCT_from is None:
                                MCT_from = -1
                            VarsList += [C[j][i][m][tau] for tau in range(
                                MCT_from + 1, t + 1) if CommWindowsBandwidth[tau][j][i] > 0 and j != i]
                            CoeffList += [1 for tau in range(
                                MCT_from + 1, t + 1) if CommWindowsBandwidth[tau][j][i] > 0 and j != i]
                    self.cpx.rows.add(1)
                    self.cpx.rows[-1].matrix = zip(VarsList, CoeffList)
                    self.cpx.rows[-1].bounds = None, 1

            self._verbprint(
                "Constraint: New information acquisition (no packet comms)")
            # New information acquisition
            for i in AgentsList:
                for m in TasksList:
                    for t in range(Thor - 1):
                        VarsList = [D[i][m][t + 1], D[i][m][t]]
                        CoeffList = [1, -1]
                        for j in AgentsList:
                            # This is the maximum time so that we will have received the information by time t+1
                            MCT = self._FindMaxStartCommTime(j, i, m, t + 1)
                            if MCT is not None:
                                VarsList += [C[j][i][m][tt]
                                             for tt in range(MCT + 1) if CommWindowsBandwidth[tt][j][i] > 0]
                                CoeffList += [-1 for tt in range(
                                    MCT + 1) if CommWindowsBandwidth[tt][j][i] > 0]
                        if t - AgentCapabilities.ComputationTime[m][i] + 1 >= 0:
                            VarsList += [X[i][m][t -
                                                 AgentCapabilities.ComputationTime[m][i] + 1]]
                            CoeffList += [-1]
                        self.cpx.rows.add(1)
                        self.cpx.rows[-1].matrix = zip(VarsList, CoeffList)
                        self.cpx.rows[-1].bounds = None, 0.

        self._verbprint("Constraint: Knowledge")
        for i in AgentsList:
            for j in AgentsList:
                for m in TasksList:
                    for t in range(Thor):
                        if CommWindowsBandwidth[t][i][j] > 0:
                            self.cpx.rows.add(1)
                            self.cpx.rows[-1].matrix = [(C[i]
                                                         [j][m][t], 1), (D[i][m][t], -1)]
                            self.cpx.rows[-1].bounds = None, 0

        self.SetupEndTime = time.time() - SetupStartTime
        self._verbprint('Setup time: {}'.format(self.SetupEndTime))
        self.X = X
        self.D = D
        self.C = C
        self.x_vars_len = x_vars_len
        self.d_vars_len = d_vars_len
        self.comm_vars_length = comm_vars_length
        self.TasksList = TasksList
        self.AgentsList = AgentsList
        self.problemIsSetUp = True

    def solve(self):
        if self.problemIsSetUp is False:
            self.setUp()
        SolveStartTime = time.time()

        if self.Verbose is False:
            glpk.env.term_on = False

        LPisFeasible = False

        start_lp = time.time()
        cpx_kwargs = {'presolve': True}
        if self.TimeLimit != 1e75:
            cpx_kwargs['tm_lim'] = int(self.TimeLimit*1e3)  # Convert to ms
        # We need this to build a feasible optimal basis. This is slow and annoying, and we will need a workaround.
        # Simplex does not respect the time limit
        retval_lp = self.cpx.simplex(**cpx_kwargs)
        end_lp = time.time() - start_lp
        if retval_lp is None and (self.cpx.status == 'opt' or self.cpx.status == 'feas'):
            # Reminder to self: intopt does not take kwargs
            start_ip = time.time()
            LPisFeasible = True
            self.cpx.integer(**cpx_kwargs)
            end_ip = time.time() - start_ip
            self._verbprint(
                'Solution status:                   %s' % self.cpx.status)

        if LPisFeasible and (self.cpx.status == 'opt' or self.cpx.status == 'feas'):
            self.opt_val = self.cpx.obj.value
            self._verbprint(
                'Optimal value:                     {}'.format(self.opt_val))
            values = [cc.value_m for cc in self.cpx.cols]
            self.values = values
            self.problemIsFeasible = True
        else:
            self.opt_val = None
            self.values = np.zeros([len(self.cpx.cols)])
        self.SolveEndTime = time.time() - SolveStartTime
        self._verbprint('Solve time: {}'.format(self.SolveEndTime))

        # This is post-processing and should probably be moved to a different function so it can be called on an existing solution.
        TaskSchedule = {}
        TaskAgent = {}

        # This is a ugly hack. Sometimes the solver will return weird fractional solutions even if we require integral ones. Here, we round.
        soleps = np.finfo(np.float32).eps
        for m in self.TasksList:
            FOUND_FLAG = False
            for t in range(self.Thor):
                for i in self.AgentsList:
                    if t in self.X[i][m].keys() and self.values[self.X[i][m][t]] >= 1. - soleps:
                        TaskSchedule[m] = t
                        TaskAgent[m] = i
                        FOUND_FLAG = True
                        break
            if not FOUND_FLAG:
                TaskSchedule[m] = None
                TaskAgent[m] = None
        TaskAssignment = {'TaskSchedule': TaskSchedule,
                          'TaskAgent': TaskAgent, 'Cost': self.opt_val}

        CommSchedule = []
        for i in self.AgentsList:
            for j in self.AgentsList:
                for m in self.TasksList:
                    for t in range(self.Thor):
                        if self.CommWindowsBandwidth[t][i][j] > 0 and self.values[self.C[i][j][m][t]] > 0:
                            if self.Options['Packet Comms'] is True:
                                CommSchedule.append([i, j, m, t])
                            else:
                                EndCommTime = self._FindEndCommTime(i, j, m, t)
                                CommSchedule.append([i, j, m, t, EndCommTime])

        RawOutput = {}
        RawOutput['X'] = self.values[0:self.x_vars_len]
        RawOutput['D'] = self.values[self.x_vars_len:self.x_vars_len + self.d_vars_len]
        RawOutput['C'] = self.values[self.x_vars_len + self.d_vars_len:]
        RawOutput['Fval'] = self.opt_val
        RawOutput['Exitflag'] = self.cpx.status
        RawOutput['Exitstring'] = self.cpx.status
        RawOutput['Setup time'] = self.SetupEndTime
        RawOutput['Total time'] = self.SetupEndTime + self.SolveEndTime
        RawOutput['GLPK problem'] = self.cpx
        self.TaskAssignment = TaskAssignment
        self.RawOutput = RawOutput
        self.CommSchedule = CommSchedule
        self.problemIsSolved = True

    def schedule(self):
        # Override to implement time limit
        startSetupTime = time.time()
        if self.problemIsSetUp is False:
            self.setUp()
        endSetupTime = time.time() - startSetupTime
        self._verbprint("Setup time (s): {}".format(endSetupTime))
        startSolveTime = time.time()
        self.solve()
        endSolveTime = time.time() - startSolveTime
        self._verbprint("Solution time: {}".format(endSolveTime))
        self._verbprint("Overall time: {}".format(endSolveTime + endSetupTime))

        if self.problemIsFeasible is False:
            print("Problem infeasible!")
            return None
        self.formatToJSON()
        return self.Timeline

    def getSolverState(self):
        if self.problemIsSolved is False:
            return "Solver not called"
        else:
            return self.cpx.status

    def getProblem(self):
        return self.cpx

    def writeSolution(self, filename='MIPstarts_GLPK'):
        self.cpx.parameters.output.writelevel.set(4)
        self.cpx.write(mps=filename + '_problem.mps')
        self.cpx.write(mip=filename + '.mps')
        self.cpx.write(sol=filename + '.sol')

    def setTimeLimits(self, DetTicksLimit=1e75, ClockTimeLimit=1e75):
        if DetTicksLimit != 1e75:
            print("WARNING: deterministic ticks limit ignored by GLPK")
        if ClockTimeLimit != 1e75:
            print("WARNING: clock time limit partly nonfunctional in GLPK")
            # pyglpk does not implement calling intop
            # (it says it does but it calls .integer)
            # The 'integer' solver requires a basis
            # To solve the basis we need a simplex solver
            #  which does not support a time limit
        self.TimeLimit = ClockTimeLimit

    def setSolverParameters(self, Properties):
        print("WARNING: parameters ignored by GLPK")
        # for param in Properties:
        #     propset = self.cpx.parameters
        #     for prop in param:
        #         propset = getattr(propset, prop)
        #     propset.set(Properties[param])
