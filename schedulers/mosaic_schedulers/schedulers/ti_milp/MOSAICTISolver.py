"""
The module ``ti_milp`` provides an implementation of the task allocation
algorithm for for robotic networks with periodic connectivity.
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
import numpy as np
import time

try:
    import cplex  # This will be problematic on ARM
    CPLEXAvailable = True
except:
    CPLEXAvailable = False

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

import json
from mosaic_schedulers.common.utilities.contact_plan_handler import compute_time_invariant_bandwidth


class MILPTasks:
    """A class containing a representation of the tasks for the MILP scheduler.

    :param OptionalTasks: a dict with Tasks as keys. OT[Task] is True iff the task
       is optional.
    :type OptionalTasks: dict
    :param TaskReward: a dict with Tasks as keys. TR[Task] is the reward obtained
       for performing the task, a float.
    :type TaskReward: dict
    :param ProductsBandwidth: a dict with Tasks as keys. PB[Task] is the bandwidth
       required to stream of the products of Task (in storage units per time
       units), a float.
    :type ProductsBandwidth: dict
    :param ProductsDataSize: a dict with Tasks as keys. PDS[Task] is the size of the
       products of Task (in storage units), a float.
    :type ProductsDataSize: dict
    :param DependencyList: a dict with Tasks as keys. DL[Task] is a list
       of tasks whose data products are required to compute Task.
    :type DependencyList: dict
    :param MaxLatency: a dict with keys task1, task2. MaxLatency[T1][T2] is the maximum
       latency that the data products of T2 (a dependency of T1) can tolerate
       before they are ingested by task T1.
    :type MaxLatency: dict
    """

    def __init__(self,
                 OptionalTasks={},
                 TaskReward={},
                 ProductsBandwidth={},
                 ProductsDataSize={},
                 DependencyList={},
                 MaxLatency={},
                 ):
        self.OptionalTasks = OptionalTasks
        self.TaskReward = TaskReward
        self.ProductsBandwidth = ProductsBandwidth
        self.ProductsDataSize = ProductsDataSize
        self.DependencyList = DependencyList
        self.MaxLatency = MaxLatency

        self.validate()

    def validate(self):
        """ Ensure the Tasks structure is valid, i.e. the keys to the various 
        components are consistent """
        assert set(self.ProductsBandwidth.keys()) == set(
            self.ProductsDataSize.keys())
        assert set(self.DependencyList.keys()) <= (
            set(self.ProductsBandwidth.keys()))
        assert set(self.DependencyList.keys()) == set(self.MaxLatency.keys())
        for task in self.MaxLatency.keys():
            assert set(self.MaxLatency[task].keys()) == set(
                self.DependencyList[task])

        if self.OptionalTasks.keys():  # If this is not empty
            assert set(self.ProductsBandwidth.keys()) == set(
                self.OptionalTasks.keys())
            assert set(self.OptionalTasks.keys()) == set(
                self.TaskReward.keys())
        else:
            for task in self.ProductsBandwidth.keys():
                self.OptionalTasks[task] = False
                self.TaskReward[task] = 0.

        self.RequiringTasks = {}
        for child in self.DependencyList.keys():
            for parent in self.DependencyList[child]:
                if parent not in self.RequiringTasks.keys():
                    self.RequiringTasks[parent] = []
                self.RequiringTasks[parent] += [child]


class MILPAgentCapabilities:
    """A class containing a representation of the agent capabilities for the
    MILP scheduler.
    
    
    :param ComputationLoad: a dictionary with keys Agent, Task. The value of
      ComputationLoad[Agent][Task] is the amount of Agent's computational
      resources required to complete Task.
    :type ComputationLoad: dict
    :param EnergyCost: a dictionary with keys Agent, Task. EnergyCost[A][T] is the
      energy cost when agent A computes task T.
    :type EnergyCost: dict
    :param MaxComputationLoad: a dict with keys Agents. MaxComputationLoad[A] is the
      maximum computational resources of agent A.
    :type MaxComputationLoad: dict
    :param LinkComputationalLoadIn: a dict with keys Agent, Agent.
      LinkComputationalLoadIn[A1][A2] is the computational load required to
      decode messages on link [A1][A2] at A2.
    :type LinkComputationalLoadIn: dict
    :param LinkComputationalLoadOut: a dict with keys Agent, Agent.
      LinkComputationalLoadOut[A1][A2] is the computational load required to
      encode messages on link [A1][A2] at A1.
    :type LinkComputationalLoadOut: dict
    """

    def __init__(self,
                 ComputationLoad={},
                 EnergyCost={},
                 MaxComputationLoad={},
                 LinkComputationalLoadIn={},
                 LinkComputationalLoadOut={},
                 ):
        self.ComputationLoad = ComputationLoad
        self.EnergyCost = EnergyCost
        self.MaxComputationLoad = MaxComputationLoad
        self.LinkComputationalLoadIn = LinkComputationalLoadIn
        self.LinkComputationalLoadOut = LinkComputationalLoadOut


class CommunicationNetwork:
    """ A class containing the properties of the communication network
    used by the time-invariant scheduler.
    
    :param Bandwidth: a dictionary with keys Agent, Agent. Bandwidth[A1][A2]
      is the bandwidth from agent A1 to agent A2. If Bandwidth[A1][A2]
      is zero, the link is not considered in the optimization problem.
    :type Bandwidth: dict
    :param Latency: a dictionary with keys Agent, Agent. Latency[A1][A2] is
      the latency of the link. Note that the latency of a datagram should
      be (but isn't at this time) computed as Latency[A1][A2] +
      datagram_size/Bandwidth[A1][A2].
    :type Latency: dict
    :param EnergyCost: a dictionary with keys Agent, Agent. EnergyCost[A1][A2]
      is the energy cost to transmit one bit on link A1-A2. The actual
      energy cost is computed as EnergyCost[A1][A2]*bit_rate[A1][A2].
    :type EnergyCost: dict
    """

    def __init__(self,
                 Bandwidth={},
                 Latency={},
                 EnergyCost={},
                 ):
        self.Bandwidth = Bandwidth
        self.Latency = Latency
        self.EnergyCost = EnergyCost


class JSONSolver:
    """
    Creates an instance of the time-invariant problem based on a problem input in the format
    described in the :doc:`API`.

    :param JSONProblemDescription: a JSON description of the problem. See the :doc:`API` for a detailed description.
    :type JSONProblemDescription: str, optional
    :param Verbose: whether to print status and debug messages. Defaults to False
    :type Verbose: bool, optional
    :param solver: the solver to use. Should be 'GLPK', 'CPLEX', or 'SCIP', defaults to 'GLPK'.
    :type solver: str, optional
    :param TimeLimit: a time limit after which to stop the optimizer. Defaults to None (no time limit).
                      Note that certain solvers (in particular, GLPK) may not honor the time limit.
    :type TimeLimit: float, optional
    :raises AssertionError: if the JSONProblemDescription is not valid.

    .. warning ::
       
       This solver does not support disjunctive prerequirements (do this task OR that task).
       If a problem with disjunctive prerequirement is passed, the solver will assert.
    
    """
    def __init__(self,
                 JSONProblemDescription,
                 Verbose=False,
                 solver='GLPK',
                 TimeLimit=None):
        if solver not in ['GLPK', 'CPLEX', 'SCIP']:
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

        if type(JSONProblemDescription) is dict:
            JSONInput = JSONProblemDescription
        else:  # Attempt to deserialize
            JSONInput = json.loads(JSONProblemDescription)

        # Time
        Thor = JSONInput["Time"]["Thor"]

        # Tasks
        # Translate tasks to TI format. Check that disjunctive requirements are not included
        DepList = {}
        for task in JSONInput["Tasks"]["DependencyList"].keys():
            if JSONInput["Tasks"]["DependencyList"][task]:
                DepList[task] = []
                for dep in JSONInput["Tasks"]["DependencyList"][task]:
                    if dep:
                        assert len(
                            dep) == 1, 'Disjunctive prerequirements are not supported yet'
                        DepList[task] += [dep[0]]
                if not DepList[task]:  # If the list is empty, pop it
                    DepList.pop(task)

        # Convert products size to average bandwidth needed
        PrBw = {}
        for task in JSONInput["Tasks"]["ProductsSize"]:
            # Average bandwidth to finish by the end of the horizon
            PrBw[task] = float(JSONInput["Tasks"]
                               ["ProductsSize"][task])/float(Thor)

        # Clean up latency by removing entries corresponding to dependencies that have been removed
        # This will ensure that the MILPTasks object is valid, specifically that DependencyList
        # and MaxLatency have the same keys
        MaxLatency = {}
        for task in JSONInput["Tasks"]["MaxLatency"]:
            if JSONInput["Tasks"]["MaxLatency"][task]:
                MaxLatency[task] = JSONInput["Tasks"]["MaxLatency"][task]

        Tasks = MILPTasks(
            OptionalTasks=JSONInput["Tasks"]["OptionalTasks"],
            TaskReward=JSONInput["Tasks"]["TaskReward"],
            ProductsDataSize=JSONInput["Tasks"]["ProductsSize"],
            ProductsBandwidth=PrBw,
            DependencyList=DepList,
            MaxLatency=MaxLatency,
        )

        # Agent capabilities

        # Convert computational load from instantaneous to average
        CompLoad = {}
        for task in JSONInput["AgentCapabilities"]["ComputationTime"].keys():
            for agent in JSONInput["AgentCapabilities"]["ComputationTime"][task].keys():
                if agent not in CompLoad.keys():
                    CompLoad[agent] = {}
                CompLoad[agent][task] = float(JSONInput["AgentCapabilities"]["ComputationLoad"][task][agent])*float(
                    JSONInput["AgentCapabilities"]["ComputationTime"][task][agent]) / float(Thor)
        TILinkLoadIn = JSONInput["AgentCapabilities"]["LinkComputationalLoadIn"]
        TILinkLoadOut = JSONInput["AgentCapabilities"]["LinkComputationalLoadOut"]
        for agent1 in TILinkLoadIn.keys():
            for agent2 in TILinkLoadIn[agent1].keys():
                TILinkLoadIn[agent1][agent2] = float(
                    TILinkLoadIn[agent1][agent2])/float(Thor)
                TILinkLoadOut[agent1][agent2] = float(
                    TILinkLoadOut[agent1][agent2])/float(Thor)

        AgentCapabilities = MILPAgentCapabilities(
            ComputationLoad=CompLoad,
            EnergyCost=flipNestedDictionary(
                JSONInput["AgentCapabilities"]["EnergyCost"]),
            MaxComputationLoad=JSONInput["AgentCapabilities"]["MaxComputationLoad"],
            LinkComputationalLoadIn=TILinkLoadIn,
            LinkComputationalLoadOut=TILinkLoadOut
        )
        # Communication network
        TVBandwidth = JSONInput["CommunicationNetwork"]

        Bandwidth, Latency, EnergyCost = compute_time_invariant_bandwidth(
            agents_list=JSONInput["AgentCapabilities"]["MaxComputationLoad"].keys(
            ),
            TVBandwidth=TVBandwidth,
            Thor=Thor)

        CommNet = CommunicationNetwork(
            Bandwidth=Bandwidth,
            Latency=Latency,
            EnergyCost=EnergyCost,
        )

        

        # Options
        Options = JSONInput["Options"]
        
        # Cost function
        if 'CostFunction' in JSONInput.keys():
            if 'CostFunction' in Options.keys():
                disp("WARNING: overriding cost function in Options with 'CostFunction'")
            Options['CostFunction'] = JSONInput['CostFunction']

        # Create the problem
        if solver == "GLPK":
            self.Scheduler = MOSAICGLPKSolver(
                AgentCapabilities=AgentCapabilities,
                Tasks=Tasks,
                CommunicationNetwork=CommNet,
                Options=Options,
                Verbose=Verbose
            )

        elif solver == "CPLEX":
            self.Scheduler = MOSAICCPLEXSolver(
                AgentCapabilities=AgentCapabilities,
                Tasks=Tasks,
                CommunicationNetwork=CommNet,
                Options=Options,
                Verbose=Verbose
            )

        elif solver == "SCIP":
            self.Scheduler = MOSAICSCIPSolver(
                AgentCapabilities=AgentCapabilities,
                Tasks=Tasks,
                CommunicationNetwork=CommNet,
                Options=Options,
                Verbose=Verbose
            )

        else:
            print("ERROR: unsupported solver (how did you get here?)")
            return

        if TimeLimit is not None:
            self.Scheduler.setTimeLimits(ClockTimeLimit=TimeLimit)

    def schedule(self):
        """ Solve the scheduling problem.

        :return: A solution in the JSON format described in the :doc:`API`.
        :rtype: str
        """
        self.Scheduler.schedule()
        return self.Scheduler.formatToBenchmarkIO()


class MOSAICMILPSolver:
    """
    .. warning ::
       
       For an easier-to-use and more consistent interface, you most likely want to call
       :class:`JSONSolver` instead of this class and its subclasses.

    Abstract implementation of the MILP task allocator.
    Subclassed by :class:`MOSAICCPLEXSolver`, :class:`MOSAICSCIPSolver`, and :class:`MOSAICGLPKSolver`.


    :param AgentCapabilities: detailing what the agents can do
    :type AgentCapabilities: MOSAICTISolver.MILPAgentCapabilities
    :param Tasks: detailing the tasks that must be achieved
    :type Tasks: MOSAICTISolver.MILPTasks
    :param CommunicationNetwork: detailing the communication network availability
    :type CommunicationNetwork: MOSAICTISolver.CommunicationNetwork
    :param Options:  additional solver-specific options, defaults to {}
    :type Options: dict, optional
    :param Verbose: if True, prints status and debug messages. Defaults to False
    :type Verbose: bool, optional
    """
    def __init__(self,
                 AgentCapabilities,
                 Tasks,
                 CommunicationNetwork,
                 Options={},
                 Verbose=False):

        self.AgentCapabilities = AgentCapabilities
        self.Tasks = Tasks
        self.CommNetwork = CommunicationNetwork
        self.Options = Options
        self.Verbose = Verbose
        self.problemIsSetUp = False
        self.problemIsSolved = False
        self.problemIsFeasible = False
        self.JSONIsFormed = False
        self.solverSpecificInit()
        self.TaskAssignment = None
        self.CommSchedule = None
        self.RawOutput = None

        for i in self.CommNetwork.Bandwidth.keys():
            if self.CommNetwork.Bandwidth[i][i] != 0:
                self.CommNetwork.Bandwidth[i][i] = 0
                self._verbprint(
                    "WARNING: removing bandwidth self-loop for agent {}".format(i))

    def solverSpecificInit(self):
        ''' A place to define solver-specific initializations in subclasses '''
        pass

    def _verbprint(self, _str):
        ''' Helper function to only print if the verbose flag is set'''
        if self.Verbose:
            print(_str)

    def getRawOutput(self):
        """ Return raw problem output from the scheduler """
        if self.problemIsSolved is False:
            self.solve()
        return self.TaskAssignment, self.CommSchedule, self.RawOutput

    def schedule(self):
        """ Set up and solve the scheduling problem """
        self.setUp()
        self.solve()
        if self.problemIsFeasible is False:
            print("Problem infeasible!")
            return (None, None)
        return (self.TaskAssignment, self.CommSchedule)

    def setTimeLimits(self, DetTicksLimit=1e75, ClockTimeLimit=1e75):
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
        """ Set up the optimization problem. Solver-specific. """
        raise NotImplementedError

    def solve(self):
        """ Solve the optimization problem. Solver-specific. """
        raise NotImplementedError

    def setMIPStart(self, MIPStart):
        """ Provide a warm start to the solver. Solver-specific. """
        raise NotImplementedError

    def getSolverState(self):
        """ Get the solver state. Solver-specific. """
        raise NotImplementedError

    def getProblem(self):
        """ Get the solver problem object. """
        raise NotImplementedError

    def getOptimizationTerminator(self):
        ''' Gets an object that can be called to stop the optimization process '''
        raise NotImplementedError

    def formatToBenchmarkIO(self):
        ''' Formats the scheduler output to the JSON format described in the :doc:`API` '''
        tasks_output = []
        if self.problemIsSolved is False:
            self._verbprint("Calling solver")
            self.solve()
        if self.problemIsFeasible is False:
            self.Timeline = None
            self._verbprint("Problem infeasible!")
            return None

        TasksList = list(self.Tasks.OptionalTasks.keys())
        # AgentsList = list(self.AgentCapabilities.ComputationLoad[TasksList[0]].keys())
        # TaskSchedule = self.TaskAssignment['TaskSchedule']
        TaskAgent = self.TaskAssignment['TaskAgent']
        # ComputationTime = None #self.AgentCapabilities.ComputationTime
        # TimeStep = self.TimeStep
        CommSchedule = self.CommSchedule

        for taskid in TasksList:
            if (TaskAgent[taskid] is not None):
                _agent = TaskAgent[taskid]
                _time = None  # float(TaskSchedule[taskid]) * TimeStep
                # float(ComputationTime[taskid][_agent]) * TimeStep
                _duration = None
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
            _time = None  # float(comm[3]) * TimeStep
            _bandwidth = float(comm[3])
            _taskname = comm[2]
            _receiver = comm[1]
            _duration = None  # product size / _bandwidth
            task_output = {
                "duration": _duration,
                "start_time": _time,
                "id": "transfer_data",
                "name": "transfer_data",
                "params": {
                    "transmitter": _agent,
                    "data_type": _taskname,
                    "agent": _agent,
                    "receiver": _receiver,
                    "bandwidth": _bandwidth,
                },
            }
            tasks_output.append(task_output)

        def sort_by_time(val):
            if val["start_time"] is None:
                return float("inf")
            else:
                return val["start_time"]
        tasks_output.sort(key=sort_by_time)
        out_dict = {"tasks": tasks_output}
        return json.dumps(out_dict)


if CPLEXAvailable is False:
    class MOSAICCPLEXSolver(MOSAICMILPSolver):
        '''
        A dummy implementation of the MILP scheduler if CPLEX is not available.
        '''

        def solverSpecificInit(self):
            print("WARNING: CPLEX not installed. This will not work.")
else:
    class MOSAICCPLEXSolver(MOSAICMILPSolver):
        """
        An implementation of the MILP scheduler that uses the IBM ILOG CPLEX solver.
        """

        def solverSpecificInit(self):

            # Create the CPLEX problem
            self.cpx = cplex.Cplex()
            self.cpx.objective.set_sense(self.cpx.objective.sense.minimize)
            self.aborter = self.cpx.use_aborter(cplex.Aborter())

        def setUp(self):
            if self.problemIsSetUp:
                return
            SetupStartTime = time.time()

            # This used to be a part of a larger function.
            # By redefining these variables as local, we save a lot of typing
            #  and possible errors.
            AgentCapabilities = self.AgentCapabilities
            Tasks = self.Tasks
            CommBandwidth = self.CommNetwork.Bandwidth
            LinkLatency = self.CommNetwork.Latency
            # Options = self.Options

            TasksList = list(Tasks.ProductsBandwidth.keys())
            AgentsList = list(AgentCapabilities.ComputationLoad.keys())

            # We find things
            ix = 0

            X = {}
            X_cost_energy = []
            X_cost_optional = []
            X_name = []
            for i in AgentsList:
                X[i] = {}
                for m in TasksList:
                    X[i][m] = ix
                    ix += 1
                    X_cost_energy.append(AgentCapabilities.EnergyCost[i][m])
                    X_cost_optional.append(
                        -float(Tasks.OptionalTasks[m])*Tasks.TaskReward[m])
                    X_name.append('X[{},{}]'.format(i, m))
            x_vars_len = ix

            B = {}
            B_name = []
            B_cost_energy = []
            B_cost_optional = []
            B_upper_bound = []
            for i in AgentsList:
                B[i] = {}
                for j in AgentsList:
                    if CommBandwidth[i][j] > 0:
                        B[i][j] = {}
                        for m in Tasks.RequiringTasks.keys():
                            B[i][j][m] = ix
                            B_name.append('B[{},{},{}]'.format(i, j, m))
                            B_cost_energy.append(
                                (self.CommNetwork.EnergyCost[i][j])*self.CommNetwork.Bandwidth[i][j])
                            B_cost_optional.append(0.)
                            B_upper_bound.append(
                                self.CommNetwork.Bandwidth[i][j])
                            ix += 1

            b_vars_len = ix - x_vars_len

            communication_cost_optional_tasks = 1e-8
            C = {}
            C_name = []
            C_cost_energy = []
            C_cost_optional = []
            C_upper_bound = []
            for i in AgentsList:
                C[i] = {}
                for j in AgentsList:
                    if CommBandwidth[i][j] > 0:
                        C[i][j] = {}
                        for m in Tasks.RequiringTasks.keys():
                            C[i][j][m] = {}
                            for mm in Tasks.RequiringTasks[m]:
                                C[i][j][m][mm] = ix
                                C_name.append(
                                    'C[{},{},{},{}]'.format(i, j, m, mm))
                                C_cost_energy.append(0)
                                C_cost_optional.append(
                                    communication_cost_optional_tasks)
                                C_upper_bound.append(
                                    self.CommNetwork.Bandwidth[i][j])
                                ix += 1

            comm_vars_length = ix - b_vars_len - x_vars_len

            if "CostFunction" in self.Options.keys():
                CostFunction = self.Options["CostFunction"]
            else:
                CostFunction = {"energy": 0.0,
                                "total_task_reward": 1.0, "total_time": 0.0}
                print("WARNING: default cost function used.")
            if CostFunction["total_time"] != 0:
                print(
                    "WARNING: minimum-makespan optimization is not supported at this time.")

            X_cost = CostFunction["total_task_reward"] * np.array(
                X_cost_optional, dtype=float) + CostFunction["energy"] * np.array(X_cost_energy, dtype=float)
            B_cost = CostFunction["total_task_reward"] * np.array(
                B_cost_optional, dtype=float) + CostFunction["energy"] * np.array(B_cost_energy, dtype=float)
            C_cost = CostFunction["total_task_reward"] * np.array(
                C_cost_optional, dtype=float) + CostFunction["energy"] * np.array(C_cost_energy, dtype=float)

            # We create the variables
            self.cpx.variables.add(obj=X_cost,
                                   lb=[0] * x_vars_len,
                                   ub=[1] * x_vars_len,
                                   types=[
                                       self.cpx.variables.type.binary] * x_vars_len,
                                   names=X_name,
                                   )

            self.cpx.variables.add(obj=B_cost,
                                   lb=[0] * b_vars_len,
                                   ub=B_upper_bound,
                                   types=[
                                       self.cpx.variables.type.continuous] * b_vars_len,
                                   names=B_name,
                                   )

            self.cpx.variables.add(
                obj=C_cost,
                lb=[0] * comm_vars_length,
                ub=C_upper_bound,
                types=[self.cpx.variables.type.continuous] * comm_vars_length,
                names=C_name,
            )

            # Constraints
            self._verbprint("Constraint: Getting the job done (1b)")
            self.cpx.linear_constraints.add(
                lin_expr=[
                    cplex.SparsePair([X[i][m] for i in AgentsList], [
                                     1.0 for i in AgentsList])
                    for m in TasksList if Tasks.OptionalTasks[m] == False],
                senses=['G' for m in TasksList if Tasks.OptionalTasks[m] == False],
                rhs=[1.0 for m in TasksList if Tasks.OptionalTasks[m] == False]
            )

            self._verbprint("Constraint: Getting the job done once")
            self.cpx.linear_constraints.add(
                lin_expr=[cplex.SparsePair([X[i][m] for i in AgentsList], [1.0 for i in AgentsList])
                          for m in TasksList],
                senses=['L' for m in TasksList],
                rhs=[1.0 for m in TasksList]
            )

            self._verbprint("Constraint: Data Transmission (1c)")
            DCConstraintsList = []
            for j in AgentsList:
                for t in Tasks.RequiringTasks.keys():
                    for tau in Tasks.RequiringTasks[t]:
                        VarsList = [X[j][t],  X[j][tau]]
                        CoeffsList = [Tasks.ProductsBandwidth[t], -
                                      Tasks.ProductsBandwidth[t]]
                        VarsList += [C[i][j][t][tau]
                                     for i in AgentsList if CommBandwidth[i][j] > 0]
                        CoeffsList += [1 for i in AgentsList if CommBandwidth[i][j] > 0]
                        VarsList += [C[j][k][t][tau]
                                     for k in AgentsList if CommBandwidth[j][k] > 0]
                        CoeffsList += [-1 for k in AgentsList if CommBandwidth[j][k] > 0]
                        DCConstraintsList += [
                            cplex.SparsePair(VarsList, CoeffsList)]
            self.cpx.linear_constraints.add(
                lin_expr=DCConstraintsList,
                senses=['G'] * len(DCConstraintsList),
                rhs=[0] * len(DCConstraintsList)
            )

            self._verbprint("Multiplexed bandwidth (1d)")
            MPConstraintsList = []
            for i in AgentsList:
                for j in AgentsList:
                    if CommBandwidth[i][j] > 0:
                        for t in Tasks.RequiringTasks.keys():
                            for tau in Tasks.RequiringTasks[t]:
                                VarsList = [B[i][j][t], C[i][j][t][tau]]
                                CoeffsList = [-1, 1]
                                MPConstraintsList += [
                                    cplex.SparsePair(VarsList, CoeffsList)]
            self.cpx.linear_constraints.add(
                lin_expr=MPConstraintsList,
                senses=['L'] * len(MPConstraintsList),
                rhs=[0] * len(MPConstraintsList)
            )

            # Resource availability
            self._verbprint("Constraint: Resource availability (1e)")
            RAConstraintList = []
            MaxAgentLoad = []
            for j in AgentsList:
                VarsList = [X[j][m] for m in TasksList]
                CoeffsList = [AgentCapabilities.ComputationLoad[j][m]
                              for m in TasksList]
                VarsList += [B[i][j][m] for i in AgentsList if CommBandwidth[i]
                             [j] > 0 for m in Tasks.RequiringTasks.keys()]
                CoeffsList += [AgentCapabilities.LinkComputationalLoadIn[i][j]
                               for i in AgentsList if CommBandwidth[i][j] > 0 for m in Tasks.RequiringTasks.keys()]
                VarsList += [B[j][k][m] for k in AgentsList if CommBandwidth[j]
                             [k] > 0 for m in Tasks.RequiringTasks.keys()]
                CoeffsList += [AgentCapabilities.LinkComputationalLoadOut[j][k]
                               for k in AgentsList if CommBandwidth[j][k] > 0 for m in Tasks.RequiringTasks.keys()]
                RAConstraintList += [cplex.SparsePair(VarsList, CoeffsList)]
                MaxAgentLoad += [AgentCapabilities.MaxComputationLoad[j]]
            self.cpx.linear_constraints.add(lin_expr=RAConstraintList,
                                            senses=['L'] *
                                            len(RAConstraintList),
                                            rhs=MaxAgentLoad)

            # self._verbprint("Constraint: Bandwidth (1f)")
            # self.cpx.linear_constraints.add(
            #     lin_expr = [
            #         cplex.SparsePair(
            #             [B[i][j][m] for m in TasksList], [1 for m in TasksList]
            #         ) for i in AgentsList for j in AgentsList if CommBandwidth[i][j]>0
            #     ],
            #     senses = ['L' for i in AgentsList for j in AgentsList if CommBandwidth[i][j]>0],
            #     rhs = [CommBandwidth[i][j]  for i in AgentsList for j in AgentsList if CommBandwidth[i][j]>0]
            # )

            self._verbprint("Constraint: Latency (1g)")
            self.cpx.linear_constraints.add(
                lin_expr=[
                    cplex.SparsePair(
                        [C[i][j][m][mm]
                            for i in AgentsList for j in AgentsList if CommBandwidth[i][j] > 0],
                        [(LinkLatency[i][j]+Tasks.ProductsDataSize[m]/self.CommNetwork.Bandwidth[i][j])/Tasks.ProductsBandwidth[m]
                            for i in AgentsList for j in AgentsList if CommBandwidth[i][j] > 0]
                    ) for mm in Tasks.DependencyList.keys() for m in Tasks.DependencyList[mm]
                ],
                senses=['L' for mm in Tasks.DependencyList.keys()
                        for m in Tasks.DependencyList[mm]],
                rhs=[Tasks.MaxLatency[mm][m] for mm in Tasks.DependencyList.keys()
                     for m in Tasks.DependencyList[mm]]
            )

            self.SetupEndTime = time.time() - SetupStartTime
            self._verbprint('Setup time: {}'.format(self.SetupEndTime))
            self.X = X
            self.B = B
            self.C = C
            self.x_vars_len = x_vars_len
            self.b_vars_len = b_vars_len
            self.comm_vars_length = comm_vars_length
            self.TasksList = TasksList
            self.AgentsList = AgentsList
            self.problemIsSetUp = True

        def solve(self):
            if self.problemIsSetUp is False:
                self.setUp()
            SolveStartTime = time.time()

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
            TaskAgent = {}

            # This is a ugly hack. Sometimes CPLEX will return weird fractional solutions even if we require integral ones. Here, we round.
            soleps = np.finfo(np.float32).eps
            for m in self.TasksList:
                FOUND_FLAG = False
                for i in self.AgentsList:
                    if self.values[self.X[i][m]] >= 1. - soleps:
                        TaskAgent[m] = i
                        FOUND_FLAG = True
                        break
                if not FOUND_FLAG:
                    TaskAgent[m] = None
            TaskAssignment = {'TaskAgent': TaskAgent, 'Cost': self.opt_val}

            CommSchedule = []
            for i in self.AgentsList:
                for j in self.AgentsList:
                    for m in self.Tasks.RequiringTasks.keys():
                        for mm in self.Tasks.RequiringTasks[m]:
                            if self.CommNetwork.Bandwidth[i][j] > 0 and self.values[self.C[i][j][m][mm]] > 0:
                                CommSchedule.append(
                                    [i, j, m, self.values[self.C[i][j][m][mm]]])

            RawOutput = {}
            RawOutput['X'] = self.values[0:self.x_vars_len]
            RawOutput['B'] = self.values[self.x_vars_len:self.x_vars_len + self.b_vars_len]
            RawOutput['C'] = self.values[self.x_vars_len + self.b_vars_len:]
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
            # If the MIP start is infeasible, how hard do we try to patch it?
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
            self._verbprint("Overall time: {}".format(
                endSolveTime + endSetupTime))

            if self.problemIsFeasible is False:
                print("Problem infeasible!")
                return (None, None)
            return (self.TaskAssignment, self.CommSchedule)

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


if GLPKAvailable is False:
    class MOSAICGLPKSolver(MOSAICMILPSolver):
        '''
        A dummy implementation of the MILP scheduler if GLPK is not available.
        '''

        def solverSpecificInit(self):
            print("WARNING: GLPK not installed. This will not work.")
else:
    class MOSAICGLPKSolver(MOSAICMILPSolver):
        """
        An implementation of the MILP scheduler that uses the GLPK solver.
        """

        def solverSpecificInit(self):
            # Create the GLPK problem
            self.cpx = glpk.LPX()
            # self.cpx.kind = int
            self.cpx.obj.maximize = False
            self.aborter = None  # No aborter for now
            self.TimeLimit = 1e75

        def setUp(self):
            if self.problemIsSetUp:
                return
            SetupStartTime = time.time()

            # This used to be a part of a larger function.
            # By redefining these variables as local, we save a lot of typing
            #  and possible errors.
            AgentCapabilities = self.AgentCapabilities
            Tasks = self.Tasks
            CommBandwidth = self.CommNetwork.Bandwidth
            LinkLatency = self.CommNetwork.Latency
            # Options = self.Options

            TasksList = list(Tasks.ProductsBandwidth.keys())
            AgentsList = list(AgentCapabilities.ComputationLoad.keys())

            # We find things
            ix = 0

            X = {}
            X_cost_energy = []
            X_cost_optional = []
            X_name = []
            for i in AgentsList:
                X[i] = {}
                for m in TasksList:
                    X[i][m] = ix
                    ix += 1
                    X_cost_energy.append(AgentCapabilities.EnergyCost[i][m])
                    X_cost_optional.append(
                        -float(Tasks.OptionalTasks[m])*Tasks.TaskReward[m])
                    X_name.append('X[{},{}]'.format(i, m))
            x_vars_len = ix

            B = {}
            B_name = []
            B_cost_energy = []
            B_cost_optional = []
            B_upper_bound = []
            for i in AgentsList:
                B[i] = {}
                for j in AgentsList:
                    if CommBandwidth[i][j] > 0:
                        B[i][j] = {}
                        for m in Tasks.RequiringTasks.keys():
                            B[i][j][m] = ix
                            B_name.append('B[{},{},{}]'.format(i, j, m))
                            B_cost_energy.append(
                                (self.CommNetwork.EnergyCost[i][j])*self.CommNetwork.Bandwidth[i][j])
                            B_cost_optional.append(0.)
                            B_upper_bound.append(
                                self.CommNetwork.Bandwidth[i][j])
                            ix += 1

            b_vars_len = ix - x_vars_len

            communication_cost_optional_tasks = 1e-8
            C = {}
            C_name = []
            C_cost_energy = []
            C_cost_optional = []
            C_upper_bound = []
            for i in AgentsList:
                C[i] = {}
                for j in AgentsList:
                    if CommBandwidth[i][j] > 0:
                        C[i][j] = {}
                        for m in Tasks.RequiringTasks.keys():
                            C[i][j][m] = {}
                            for mm in Tasks.RequiringTasks[m]:
                                C[i][j][m][mm] = ix
                                C_name.append(
                                    'C[{},{},{},{}]'.format(i, j, m, mm))
                                C_cost_energy.append(0)
                                C_cost_optional.append(
                                    communication_cost_optional_tasks)
                                C_upper_bound.append(
                                    self.CommNetwork.Bandwidth[i][j])
                                ix += 1

            comm_vars_length = ix - b_vars_len - x_vars_len

            if "CostFunction" in self.Options.keys():
                CostFunction = self.Options["CostFunction"]
            else:
                CostFunction = {"energy": 0.0,
                                "total_task_reward": 1.0, "total_time": 0.0}
                print("WARNING: default cost function used.")
            if CostFunction["total_time"] != 0:
                print(
                    "WARNING: minimum-makespan optimization is not supported at this time.")

            X_cost = CostFunction["total_task_reward"] * np.array(
                X_cost_optional, dtype=float) + CostFunction["energy"] * np.array(X_cost_energy, dtype=float)
            B_cost = CostFunction["total_task_reward"] * np.array(
                B_cost_optional, dtype=float) + CostFunction["energy"] * np.array(B_cost_energy, dtype=float)
            C_cost = CostFunction["total_task_reward"] * np.array(
                C_cost_optional, dtype=float) + CostFunction["energy"] * np.array(C_cost_energy, dtype=float)

            # We create the variables

            self.cpx.cols.add(len(X_cost))
            for col_ix in range(len(X_cost)):
                c = self.cpx.cols[col_ix]
                # c.bounds = 0., 1.
                c.kind = bool
                c.name = X_name[col_ix]
                self.cpx.obj[col_ix] = X_cost[col_ix]

            if len(B_cost):
                self.cpx.cols.add(len(B_cost))
                for col_ix in range(len(B_cost)):
                    c = self.cpx.cols[col_ix+len(X_cost)]
                    c.kind = float
                    c.bounds = 0., B_upper_bound[col_ix]
                    c.name = B_name[col_ix]
                    self.cpx.obj[col_ix+len(X_cost)] = B_cost[col_ix]

            if len(C_cost):
                self.cpx.cols.add(len(C_cost))
                for col_ix in range(len(C_cost)):
                    c = self.cpx.cols[col_ix+len(X_cost)+len(B_cost)]
                    c.kind = float
                    c.bounds = 0., C_upper_bound[col_ix]
                    c.name = C_name[col_ix]
                    self.cpx.obj[col_ix+len(X_cost) +
                                 len(B_cost)] = C_cost[col_ix]

            # Constraints
            self._verbprint("Constraint: Getting the job done (1b)")
            for m in TasksList:
                if Tasks.OptionalTasks[m] == False:
                    self.cpx.rows.add(1)
                    self.cpx.rows[-1].matrix = [(X[i][m], 1.0)
                                                for i in AgentsList]
                    self.cpx.rows[-1].bounds = 1.0, None

            self._verbprint("Constraint: Getting the job done once")
            for m in TasksList:
                self.cpx.rows.add(1)
                self.cpx.rows[-1].matrix = [(X[i][m], 1.) for i in AgentsList]
                self.cpx.rows[-1].bounds = None, 1.

            self._verbprint("Constraint: Data Transmission (1c)")
            for j in AgentsList:
                for t in Tasks.RequiringTasks.keys():
                    for tau in Tasks.RequiringTasks[t]:
                        VarsList = [X[j][t],  X[j][tau]]
                        CoeffsList = [Tasks.ProductsBandwidth[t], -
                                      Tasks.ProductsBandwidth[t]]
                        VarsList += [C[i][j][t][tau]
                                     for i in AgentsList if CommBandwidth[i][j] > 0]
                        CoeffsList += [1 for i in AgentsList if CommBandwidth[i][j] > 0]
                        VarsList += [C[j][k][t][tau]
                                     for k in AgentsList if CommBandwidth[j][k] > 0]
                        CoeffsList += [-1 for k in AgentsList if CommBandwidth[j][k] > 0]
                        self.cpx.rows.add(1)
                        self.cpx.rows[-1].matrix = zip(VarsList, CoeffsList)
                        self.cpx.rows[-1].bounds = 0., None

            self._verbprint("Multiplexed bandwidth (1d)")
            for i in AgentsList:
                for j in AgentsList:
                    if CommBandwidth[i][j] > 0:
                        for t in Tasks.RequiringTasks.keys():
                            for tau in Tasks.RequiringTasks[t]:
                                VarsList = [B[i][j][t], C[i][j][t][tau]]
                                CoeffsList = [-1, 1]
                                self.cpx.rows.add(1)
                                self.cpx.rows[-1].matrix = zip(VarsList,
                                                               CoeffsList)
                                self.cpx.rows[-1].bounds = None, 0.

            # Resource availability
            self._verbprint("Constraint: Resource availability (1e)")
            for j in AgentsList:
                VarsList = [X[j][m] for m in TasksList]
                CoeffsList = [AgentCapabilities.ComputationLoad[j][m]
                              for m in TasksList]
                VarsList += [B[i][j][m] for i in AgentsList if CommBandwidth[i]
                             [j] > 0 for m in Tasks.RequiringTasks.keys()]
                CoeffsList += [AgentCapabilities.LinkComputationalLoadIn[i][j]
                               for i in AgentsList if CommBandwidth[i][j] > 0 for m in Tasks.RequiringTasks.keys()]
                VarsList += [B[j][k][m] for k in AgentsList if CommBandwidth[j]
                             [k] > 0 for m in Tasks.RequiringTasks.keys()]
                CoeffsList += [AgentCapabilities.LinkComputationalLoadOut[j][k]
                               for k in AgentsList if CommBandwidth[j][k] > 0 for m in Tasks.RequiringTasks.keys()]
                self.cpx.rows.add(1)
                self.cpx.rows[-1].matrix = zip(VarsList, CoeffsList)
                self.cpx.rows[-1].bounds = None, AgentCapabilities.MaxComputationLoad[j]

            self._verbprint("Constraint: Latency (1g)")
            for mm in Tasks.DependencyList.keys():
                for m in Tasks.DependencyList[mm]:
                    self.cpx.rows.add(1)
                    self.cpx.rows[-1].matrix = [(C[i][j][m][mm], (LinkLatency[i][j]+Tasks.ProductsDataSize[m]/self.CommNetwork.Bandwidth[i][j])/Tasks.ProductsBandwidth[m])
                                                for i in AgentsList for j in AgentsList if CommBandwidth[i][j] > 0]
                    self.cpx.rows[-1].bounds = None, Tasks.MaxLatency[mm][m]

            self.SetupEndTime = time.time() - SetupStartTime
            self._verbprint('Setup time: {}'.format(self.SetupEndTime))
            self.X = X
            self.B = B
            self.C = C
            self.x_vars_len = x_vars_len
            self.b_vars_len = b_vars_len
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
            # We need this to build a feasible optimal basis. This is slow and annoying, and we will need a workaround.
            # retval_lp = self.cpx.interior()
            start_lp = time.time()
            self._verbprint("Solving simplex")
            cpx_kwargs = {'presolve': True}
            if self.TimeLimit != 1e75:
                cpx_kwargs['tm_lim'] = int(self.TimeLimit*1e3)  # Convert to ms
            retval_lp = self.cpx.simplex(**cpx_kwargs)
            end_lp = time.time() - start_lp
            self._verbprint("Simplex solved in {} s".format(end_lp))

            if retval_lp is None and (self.cpx.status == 'opt' or self.cpx.status == 'feas'):
                self._verbprint("Solving IP")
                LPisFeasible = True

                start_ip = time.time()
                self.cpx.integer(**cpx_kwargs)
                end_ip = time.time() - start_ip
                self._verbprint("MILP solved in {} s".format(end_ip))

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
            TaskAgent = {}

            # This is a ugly hack. Sometimes the solver will return weird fractional solutions even if we require integral ones. Here, we round.
            soleps = np.finfo(np.float32).eps
            for m in self.TasksList:
                FOUND_FLAG = False
                for i in self.AgentsList:
                    if self.values[self.X[i][m]] >= 1. - soleps:
                        TaskAgent[m] = i
                        FOUND_FLAG = True
                        break
                if not FOUND_FLAG:
                    TaskAgent[m] = None
            TaskAssignment = {'TaskAgent': TaskAgent, 'Cost': self.opt_val}

            CommSchedule = []
            for i in self.AgentsList:
                for j in self.AgentsList:
                    for m in self.Tasks.RequiringTasks.keys():
                        for mm in self.Tasks.RequiringTasks[m]:
                            if self.CommNetwork.Bandwidth[i][j] > 0 and self.values[self.C[i][j][m][mm]] > 0:
                                CommSchedule.append(
                                    [i, j, m, self.values[self.C[i][j][m][mm]]])

            RawOutput = {}
            RawOutput['X'] = self.values[0:self.x_vars_len]
            RawOutput['B'] = self.values[self.x_vars_len:self.x_vars_len + self.b_vars_len]
            RawOutput['C'] = self.values[self.x_vars_len + self.b_vars_len:]
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
            startSolveTime = time.time()
            self.solve()
            endSolveTime = time.time() - startSolveTime
            self._verbprint("Solution time: {}".format(endSolveTime))
            self._verbprint("Overall time: {}".format(
                endSolveTime + endSetupTime))

            if self.problemIsFeasible is False:
                print("Problem infeasible!")
                return (None, None)
            return (self.TaskAssignment, self.CommSchedule)

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

if SCIPAvailable is False:
    class MOSAICSCIPSolver(MOSAICMILPSolver):
        '''
        A dummy implementation of the MILP scheduler if SCIP is not available.
        '''

        def solverSpecificInit(self):
            print("WARNING: SCIP not installed. This will not work.")
else:
    class MOSAICSCIPSolver(MOSAICMILPSolver):
        """
        An implementation of the MILP scheduler that uses the SCIP solver.
        """

        def solverSpecificInit(self):
            # # Create the SCIP problem
            self.cpx = scip.Model("ti-milp")
            if self.Verbose is False:
                self.cpx.hideOutput()
            self.aborter = None  # No aborter for now
            self.TimeLimit = None

        def setUp(self):
            if self.problemIsSetUp:
                return
            SetupStartTime = time.time()

            # This used to be a part of a larger function.
            # By redefining these variables as local, we save a lot of typing
            #  and possible errors.
            AgentCapabilities = self.AgentCapabilities
            Tasks = self.Tasks
            CommBandwidth = self.CommNetwork.Bandwidth
            LinkLatency = self.CommNetwork.Latency
            # Options = self.Options

            TasksList = list(Tasks.ProductsBandwidth.keys())
            AgentsList = list(AgentCapabilities.ComputationLoad.keys())

            # Set up cost function
            if "CostFunction" in self.Options.keys():
                CostFunction = self.Options["CostFunction"]
            else:
                CostFunction = {"energy": 0.0,
                                "total_task_reward": 1.0, "total_time": 0.0}
                print("WARNING: default cost function used.")
            if CostFunction["total_time"] != 0:
                print(
                    "WARNING: minimum-makespan optimization is not supported at this time.")

            # We create variables
            ix = 0
            X = {}
            X_cost = {}
            for i in AgentsList:
                X[i] = {}
                X_cost[i] = {}
                for m in TasksList:
                    ix += 1
                    X[i][m] = self.cpx.addVar(
                        vtype="B",
                        name='X[{},{}]'.format(i, m)
                    )
                    X_cost[i][m] = -CostFunction["total_task_reward"]*(float(
                        Tasks.OptionalTasks[m])*Tasks.TaskReward[m]) + CostFunction["energy"]*float(AgentCapabilities.EnergyCost[i][m])
            x_vars_len = ix

            B = {}
            B_cost = {}
            for i in AgentsList:
                B[i] = {}
                B_cost[i] = {}
                for j in AgentsList:
                    if CommBandwidth[i][j] > 0:
                        B[i][j] = {}
                        B_cost[i][j] = {}
                        for m in Tasks.RequiringTasks.keys():
                            B[i][j][m] = self.cpx.addVar(
                                vtype="C",
                                lb=0,
                                ub=self.CommNetwork.Bandwidth[i][j],
                                name='B[{},{},{}]'.format(i, j, m)
                            )
                            B_cost[i][j][m] = CostFunction["energy"]*float(
                                (self.CommNetwork.EnergyCost[i][j])*self.CommNetwork.Bandwidth[i][j])
                            ix += 1

            b_vars_len = ix - x_vars_len

            communication_cost_optional_tasks = 1e-8
            C = {}
            C_cost = {}
            for i in AgentsList:
                C[i] = {}
                C_cost[i] = {}
                for j in AgentsList:
                    if CommBandwidth[i][j] > 0:
                        C[i][j] = {}
                        C_cost[i][j] = {}
                        for m in Tasks.RequiringTasks.keys():
                            C[i][j][m] = {}
                            C_cost[i][j][m] = {}
                            for mm in Tasks.RequiringTasks[m]:
                                C[i][j][m][mm] = self.cpx.addVar(
                                    vtype="C",
                                    lb=0,
                                    ub=self.CommNetwork.Bandwidth[i][j],
                                    name='C[{},{},{},{}]'.format(i, j, m, mm)
                                )
                                C_cost[i][j][m][mm] = CostFunction["total_task_reward"] * \
                                    (communication_cost_optional_tasks)
                                ix += 1

            comm_vars_length = ix - b_vars_len - x_vars_len

            # We set the objective
            self.cpx.setObjective(
                scip.quicksum(X_cost[i][m] * X[i][m]
                              for i in AgentsList for m in TasksList)
                + scip.quicksum(B_cost[i][j][m]*B[i][j][m]
                                for i in AgentsList for j in AgentsList for m in Tasks.RequiringTasks.keys() if CommBandwidth[i][j] > 0)
                + scip.quicksum(C_cost[i][j][m][mm] * C[i][j][m][mm]
                                for i in AgentsList for j in AgentsList for m in Tasks.RequiringTasks.keys() for mm in Tasks.RequiringTasks[m] if CommBandwidth[i][j] > 0),
                "minimize")

            # Constraints
            self._verbprint("Constraint: Getting the job done (1b)")
            for m in TasksList:
                if Tasks.OptionalTasks[m] == False:
                    self.cpx.addCons(scip.quicksum(
                        X[i][m] for i in AgentsList) >= 1.)

            self._verbprint("Constraint: Getting the job done once")
            for m in TasksList:
                self.cpx.addCons(scip.quicksum(
                    X[i][m] for i in AgentsList) <= 1.)

            self._verbprint("Constraint: Data Transmission (1c)")
            # DCConstraintsList = []
            for j in AgentsList:
                for t in Tasks.RequiringTasks.keys():
                    for tau in Tasks.RequiringTasks[t]:
                        self.cpx.addCons(
                            Tasks.ProductsBandwidth[t]*X[j][t] -
                            Tasks.ProductsBandwidth[t]*X[j][tau]
                            + scip.quicksum(C[i][j][t][tau]
                                            for i in AgentsList if CommBandwidth[i][j] > 0)
                            - scip.quicksum(C[j][k][t][tau]
                                            for k in AgentsList if CommBandwidth[j][k] > 0)
                            >= 0.
                        )

            self._verbprint("Multiplexed bandwidth (1d)")
            # MPConstraintsList = []
            for i in AgentsList:
                for j in AgentsList:
                    if CommBandwidth[i][j] > 0:
                        for t in Tasks.RequiringTasks.keys():
                            for tau in Tasks.RequiringTasks[t]:
                                self.cpx.addCons(-B[i][j][t] +
                                                 C[i][j][t][tau] <= 0)

            # Resource availability
            self._verbprint("Constraint: Resource availability (1e)")
            # RAConstraintList = []
            # MaxAgentLoad = []
            for j in AgentsList:
                self.cpx.addCons(
                    scip.quicksum(AgentCapabilities.ComputationLoad[j][m]*X[j][m] for m in TasksList) +
                    scip.quicksum(AgentCapabilities.LinkComputationalLoadIn[i][j]*B[i][j][m] for i in AgentsList if CommBandwidth[i][j] > 0 for m in Tasks.RequiringTasks.keys()) +
                    scip.quicksum(AgentCapabilities.LinkComputationalLoadOut[j][k]*B[j][k][m]
                                  for k in AgentsList if CommBandwidth[j][k] > 0 for m in Tasks.RequiringTasks.keys())
                    <= AgentCapabilities.MaxComputationLoad[j]
                )

            self._verbprint("Constraint: Latency (1g)")
            for mm in Tasks.DependencyList.keys():
                for m in Tasks.DependencyList[mm]:
                    self.cpx.addCons(
                        scip.quicksum(
                            (LinkLatency[i][j]+Tasks.ProductsDataSize[m]/self.CommNetwork.Bandwidth[i][j])/Tasks.ProductsBandwidth[m]*C[i][j][m][mm] for i in AgentsList for j in AgentsList if CommBandwidth[i][j] > 0
                        )
                        <= Tasks.MaxLatency[mm][m]
                    )

            self.SetupEndTime = time.time() - SetupStartTime
            self._verbprint('Setup time: {}'.format(self.SetupEndTime))
            self.X = X
            self.B = B
            self.C = C
            self.x_vars_len = x_vars_len
            self.b_vars_len = b_vars_len
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
            TaskAgent = {}

            # This is a ugly hack. Sometimes SCIP will return weird fractional solutions even if we require integral ones. Here, we round.
            soleps = np.finfo(np.float32).eps
            for m in self.TasksList:
                FOUND_FLAG = False
                for i in self.AgentsList:
                    if self.cpx.getVal(self.X[i][m]) >= 1. - soleps:
                        TaskAgent[m] = i
                        FOUND_FLAG = True
                        break
                if not FOUND_FLAG:
                    TaskAgent[m] = None
            TaskAssignment = {'TaskAgent': TaskAgent, 'Cost': self.opt_val}

            CommSchedule = []
            for i in self.AgentsList:
                for j in self.AgentsList:
                    for m in self.Tasks.RequiringTasks.keys():
                        for mm in self.Tasks.RequiringTasks[m]:
                            if self.CommNetwork.Bandwidth[i][j] > 0 and self.cpx.getVal(self.C[i][j][m][mm]) > 0:
                                CommSchedule.append(
                                    [i, j, m, self.cpx.getVal(self.C[i][j][m][mm])])

            RawOutput = {}
            RawOutput['X'] = self.X
            RawOutput['B'] = self.B
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
                self._verbprint(
                    "Overall time limit: {}".format(overallTimeLimit))
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
            self._verbprint("Overall time: {}".format(
                endSolveTime + endSetupTime))

            if self.problemIsFeasible is False:
                print("Problem infeasible!")
                return (None, None)
            return (self.TaskAssignment, self.CommSchedule)

        def getSolverState(self):
            if self.problemIsSolved is False:
                return "Solver not called"
            else:
                return self.cpx.getStatus()

        def getProblem(self):
            return self.cpx

        # def writeSolution(self, filename='MIPstarts'):
        #     # TODO
        #     self.cpx.parameters.output.writelevel.set(4)
        #     self.cpx.MIP_starts.write(filename + '.mps')
        #     self.cpx.solution.write(filename + '.sol')

        def setTimeLimits(self, DetTicksLimit=None, ClockTimeLimit=None):
            if DetTicksLimit is not None:
                print("WARNING: deterministic ticks limit ignored by SCIP")
            if ClockTimeLimit is not None:
                self.TimeLimit = ClockTimeLimit

        def setSolverParameters(self, Properties):
            for param in Properties:
                self.cpx.setRealParam(param, Properties[param])


def flipNestedDictionary(_dictionary):
    # A utility function to flip the keys of a dictionary. No error handling is deliberate:
    # this should only take as inputs dicts where all entries have the same (sub)keys.
    newDict = {}
    keys1 = list(_dictionary.keys())
    if len(keys1):
        keys2 = _dictionary[keys1[0]].keys()
        for keyA in keys2:
            newDict[keyA] = {}
            for keyB in keys1:
                newDict[keyA][keyB] = _dictionary[keyB][keyA]
    return newDict
