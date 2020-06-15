.. _time_varying:

Time-varying scheduling algorithm
=================================

We describe the communication-aware, data-driven computation task
scheduling problem for heterogeneous multi-robot systems.

A full description of the algorithm is available in `our paper <#references>`__.

Problem Description
-------------------

Tasks and Software Network
^^^^^^^^^^^^^^^^^^^^^^^^^^

We wish to schedule :math:`M\in\mathbb{Z}^{+}` data-driven tasks, where
:math:`\mathbb{Z}^{+}` denotes positive integers. The set of :math:`M`
tasks is denoted :math:`\mathbb{T}`. Computational tasks of interest can
include, e.g. localizing a robot, computing a motion plan for a robot,
classifying and labeling the content of an image
, or estimating the
spatial distribution of a phenomenon based on point measurements from
multiple assets.

Tasks may be *required* or *optional*. Required tasks, denoted as
:math:`{\mathbb{R}}\subseteq{\mathbb{T}}`, must be
included in the schedule. Optional tasks
:math:`T\in {\mathbb{T}}\setminus {\mathbb{R}}`
are each assigned a *reward* score :math:`r(T)`, which denotes the value
of including the task in a schedule.

The output of each task is a *data product*. Data products for task
:math:`T` are denoted as :math:`d({T})`. The size (in bits) of the data
products are known a-priori as :math:`s\left({T}\right)` for task
:math:`T`.

Tasks are connected by dependency relations encoded in a *software
network* :math:`SN`. Let :math:`P_T` be a set of predecessor tasks for
:math:`T`. If :math:`\hat T \in P_T`, task :math:`T` can only be
executed by a robot if the robot has data product
:math:`{d({\hat T})}`. If :math:`\hat T` is scheduled to be
executed on the same robot as :math:`T`,
:math:`{d({\hat T})}` is assumed to be available to :math:`T`
as soon as the computation of :math:`\hat T` is concluded. If
:math:`\hat T` and :math:`T` are scheduled on different robots,
:math:`{d({\hat T})}` must be transmitted from the robot
executing :math:`\hat T` to the robot executing :math:`T` before
execution of :math:`T` can commence. An example of :math:`SN` used in
our experiments is shown below.

.. image:: ./images/scenario_task_network.png
  :alt: Sample task network

The following two assumptions ensure that a solution to the scheduling
problem exists.

**Assumption: feasibility** There exists a schedule where all
required tasks are scheduled.

The software network :math:`SN` do not have cycles.

Agents
^^^^^^

Agents in the network represent computing units. Let there be
:math:`N\in\mathbb{Z}^{+}` agents in the network. The agents are denoted
by
:math:`{A_{1}},\thinspace {A_{2}},\thinspace\ldots,\thinspace {A_{N}}`.
Each agent has known on-board processing and storage capabilities.

The time and energy cost required to perform a task :math:`T` on agent
:math:`i` are assumed to be known and denoted respectively as
:math:`{\tau_{i}\left({T}\right)}` and
:math:`{C_{i}^e\left({T}\right)}`. Depending on the
application, time and energy cost can capture the worst-case, expected,
or bounded computation time and energy; they are all considered to be
deterministic.

.. .. figure:: contact_graph
..    :alt:  Contact graph for :math:`3` agents showing connectivity time
..    windows and bandwidths available. [fig:Contact-graph]
..    :name: fig:Contact-graph

..    Contact graph for :math:`3` agents showing connectivity time windows
..    and bandwidths available. [fig:Contact-graph]

.. _sec:cg:

Contact Graph
^^^^^^^^^^^^^

Agents can communicate according to a prescribed time-varying *contact
graph* :math:`CG` which denotes the availability and bandwidth of
communication links between the robots.

:math:`CG` is a graph with time-varying edges. Nodes :math:`\mathcal{V}`
in :math:`CG` correspond to agents. For each time instant :math:`k`,
directed edges :math:`\mathcal{E}_{k}` model the availability of
communication links; that is, :math:`(i, j) \in \mathcal{E}_{k}` if node
:math:`i` can communicate to node :math:`j` at time :math:`k`. Each edge
has a (time-varying) data rate ranging from :math:`0` (not connected) to
:math:`\infty` (communicating to self), denoted by
:math:`{r_{{i}{j}}({k})}` for the rate from :math:`A_{i}` to
:math:`A_{j}` at time :math:`k`. An example timeline representation for
:math:`3` agents with available bandwidths can be seen in the
figure below.

.. image:: ./images/contact_graph.png
  :alt: Contact graph for :math:`3` agents showing connectivity time windows and bandwidths available.


A key feature of DTN-based networking is Contact Graph Routing (CGR).
CGR takes into account
predictable link schedules and bandwidth limits to automate data
delivery and optimize the use of network resources. Accordingly, by
incorporating DTN’s store-forward mechanism into the scheduling problem,
it is possible to use mobile agents as *robotic routers* to ferry data
packets between agents that are not directly connected.

Communicating the data product :math:`d({T})` from :math:`A_{i}` to
:math:`A_{j}` at time :math:`k` requires time

.. math:: {\tau_{ij}\left({T}\right)}: \arg\min_{\tau}= \int_{k}^{k+\tau} {r_{{i}{j}}({\hat k})} d \hat k = {s\left({T}\right)}.

The following two assumptions model availability of computational
resources and the time required to share data products between processes
on a single robot.

**Assumption: no task concurrency** Agents can only perform a single task at any
given time, including transmitting or receiving data products.

Agents take :math:`0` time to communicate the solution to themselves.

Schedule
^^^^^^^^

A schedule is (a) a mapping of tasks to agents and start-times, denoted
as :math:`{\mathbb{S}}:T\rightarrow ({A_{i}}, k)`
where :math:`i\in[1,\ldots,N]` and :math:`k\geq0`, and (b) a list of
inter-agent communications
:math:`({A_{i}}, {A_{j}}, {d({T})}, k)`
denoting the transmission of :math:`{d({T})}` from
:math:`{A_{i}}` to :math:`{A_{j}}` from time
:math:`k` to time
:math:`\bar k : \left(\int_{k}^{\bar k} {r_{{i}{j}}({\kappa})} d \kappa = {s\left({T}\right)}\right)`.

Optimization Objectives
^^^^^^^^^^^^^^^^^^^^^^^

We consider several optimization objectives (formalized in the following
section), including:

-  *Optional tasks*: maximize the sum of the rewards :math:`r(T)` for
   optional tasks :math:`T` that are included in the schedule;

-  *Makespan*: minimize the maximum completion time of all scheduled
   tasks;

-  *Energy cost*: minimize the sum of the energy costs
   :math:`{C_{i}^e\left({T}\right)}` for tasks included in
   the schedule;


.. _scheduling_problem_def:

Scheduling Problem
^^^^^^^^^^^^^^^^^^

We are now in a position to state the communication-aware, data-driven
computation task scheduling problem for heterogeneous multi-robot
systems.

**Communication-aware, data-driven computation task scheduling problem for heterogeneous multi-robot systems** Given a set of tasks modeled as a software network
:math:`SN`, a list of computational agents :math:`A_{i}`,
:math:`i\in[1\ldots N]`, a contact graph :math:`CG`, and a maximum
schedule length :math:`C^\star`, find a schedule that satisfies:

#. The maximum overall computation time is no more than :math:`C^\star`;

#. All required tasks :math:`T\in \mathbb{R}` are scheduled;

#. A task :math:`T` is only scheduled on agent
   :math:`{A_{i}}` at time :math:`k` if the agent has
   received all the data product :math:`{d({\hat T})}` for
   predecessor tasks :math:`\hat T \in P_T`;

#. Every agent performs at most one task (including transmitting and
   receiving data products) at any time;

#. The selected optimization objective is maximized.


Notes on Problem Assumptions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The assumption that a feasible schedule including all required tasks
exists is appropriate for multi-robot systems where each required task
“belongs” to a specific robot (i.e., the task is performed with inputs
collected by the robot, and the output of the task is to be consumed by
the same robot). Examples of such tasks include localization, mapping,
and path planning. In such a setting, it is reasonable to assume that
each robot should be able to perform all of its own required tasks with
no assistance from other computation nodes; on the other hand,
cooperation between robots can decrease the makespan, reduce energy use,
and enable the completion of optional tasks.

The contact graph is assumed to be known in advance. This assumption is
reasonable in many space applications, specifically in surface-to-orbit
communications, orbit-to-orbit communications, and surface-to-surface
communication in unobstructed environments, where the capacity of the
communication channel can be predicted to a high degree of accuracy. In
obstructed environments where communication models are highly uncertain
(e.g. subsurface voids such as caves, mines, tunnels) a conservative
estimate of the channel capacity could be used. Extending the scheduling
problem to explicitly capture uncertainty in the communication graph is
an interesting direction for future research.

Finally, the scheduling problem also assumes that the
communication graph is not part of the optimization process. The problem
of optimizing the contact graph by prescribing the agents’ motion is
beyond the scope of this paper; however, we remark that the tools in
this paper can be used as an optimization subroutine to numerically
assess the effect of proposed changes in the contact graph on the
performance of the multi-robot system.

.. _sec_scheduler_ilp:

ILP formulation
---------------

We formulate the :ref:`scheduling_problem_def` as an integer linear
program (ILP). We consider a discrete-time approximation of the problem
with a time horizon of :math:`{C^\star_d}` time steps, each
of duration :math:`C^\star/{C^\star_d}`, corresponding to the
maximum schedule length :math:`C^\star`. The optimization variables are:

-  :math:`X`, a set of Boolean variables of size
   :math:`N\cdot M \cdot {C^\star_d}`. :math:`X(i,T,k)` is
   true if and only if agent :math:`A_i` starts computing task :math:`T`
   at time :math:`k`.

-  :math:`D`, a set of Boolean variables of size
   :math:`N \cdot M \cdot {C^\star_d}`. :math:`D(i,T,k)`, is
   true if and only if agent :math:`A_i` has stored the data products
   :math:`d(T)` of task :math:`T` at time :math:`k`.

-  :math:`C`, a set of Boolean variables of size
   :math:`N^2 \cdot M \cdot {C^\star_d}`. :math:`C(i,j,T,k)`,
   is true if and only if agent :math:`A_i` communicates part or all of
   data products :math:`{d({T})}` to agent :math:`A_j` at
   time :math:`k`.

The optimization objective :math:`R` can be expressed as follows:

-  Maximize the sum of the rewards for completed optional tasks:

We are now in a position to formally state the ILP formulation of
the problem:

.. math::

   \begin{aligned}
   & \underset{X, D, C}{\text{maximize }} R \\
   %
   & \text{subject to}\nonumber \\
   %
   &  \sum_{i=1}^N   \sum_{k=1}^{{C^\star_d}-{\tau_{i}\left({T}\right)}} X(i,T,k)  = 1 \quad \forall T\in \mathbb{R} \label{eq:MILP:requiredtasks}\\
   %
   &  \sum_{i=1}^N \sum_{k=1}^{{C^\star_d}-{\tau_{i}\left({T}\right)}} X(i,T,k)  \leq 1  \quad \forall T\in\mathbb{T}\setminus \mathbb{R} \label{eq:MILP:optionaltasks}\\
   %
   &  X(i,T,k) \leq D(i,L,k) \label{eq:MILP:prereqs} \quad  \forall i\in[1,\ldots,N], T\in[1,\ldots,M], L\in P_T, k\in[1,\ldots,{C^\star_d}] \nonumber \\
   %
   & \sum_{T=1}^M \left[ \sum_{j=1}^N\left( C(i,j,T,k) + C(j,i,T,k) \right) + \sum_{{\hat k= \max(1,k-{\tau_{i}\left({T}\right)})}}^kX(i,T,\hat k) \right] \leq 1 \nonumber \\
   & \quad \quad \forall i\in[1,\ldots,N], k\in[1,\ldots,{C^\star_d}] \label{eq:MILP:computation}\\
   %
   & D(i,T,k+1)\!-\!D(i,T,k) \leq \sum_{\tau=1}^k\sum_{j=1}^N \frac{r_{ji}(k)}{{s\left({T}\right)}} C(j,i,T,k) + \sum_{\tau=1}^{{k-{\tau_{i}\left({T}\right)}}} X(i,T,k) \nonumber \\
   & \quad \quad \forall i\in[1,\ldots,N],  T\in[1,\ldots,M], k\in[1,\ldots,{C^\star_d}-1] \label{eq:MILP:learning}\\
   %
   & C(i,j,T,k) \leq D(i,T,k) \quad  \forall  i,j \in[1,\ldots,N], T\in[1,\ldots,M], k\in[1,\ldots,T] \label{eq:MILP:knowledge}\\
   %
   & D(i,T,1)=0 \quad \forall i \in[1,\ldots,N], T\in[1,\ldots,M] \label{eq:MILP:initialinfo}\end{aligned}


The first constraint ensures
that all required tasks are performed, and the second constraint that
optional tasks are performed at most once.
The third constraint requires that agents only start
a task if they have access to the data products of all its predecessor
tasks.
The fourth constraint captures the agents’ limited computation resources
by enforcing the no-concurrency assumption.
The fifth constraint  ensures that agents
learn the content of a task’s data products only if they (i) receive
such information from other agents (possibly over multiple time steps,
each carrying a fraction
:math:`r_{ij}(k)/{s\left({T}\right)}` of the data product) or
(ii) complete the task themselves.
The sixth constraint ensures that agent only
communicate a data product if they have stored the data product
themselves. Finally, the seventh and last constraint models the fact that
data products are initially unknown to all agents.

The ILP has :math:`N^2M{C^\star_d}+2NM{C^\star_d}`
Boolean variables and
:math:`M(N(3{C^\star_d}-1)+N )+N{C^\star_d}`
constraints; instances with dozens of agents and tasks and horizons of
50–100 time steps can be readily solved by state-of-the-art ILP solvers.

Modeling Extensions: Modeling Network Interference
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :ref:`sec_scheduler_ilp` is amenable to several
modeling extensions. In this section, we show how the bandwidth
constraints can be modified to capture network interference caused by
the agents’ communications, a critical consideration for close-range
operations.

In the formulation, link bandwidths :math:`r_{ij}` are assumed
to be fixed and independent of each other: that is, the communication
bandwidth :math:`r_{ij}` on a link is assumed to be achievable
regardless of communication activity on other links. This assumption can
be inaccurate for systems where many robots operate in close proximity
and share the same wireless channel; in such a setting, interference
introduces a coupling between the achievable bandwidths on different
links, and the overall amount of data that can be exchanged by
interfering links is limited by the *channel capacity* of the shared
physical medium.

The formulation can be extended to capture a
first-order approximation of this effect, letting individual link bit
rates be decision variables subject to constraints on the overall
channel capacity.

Effectively, agents are allowed to use less than the full capacity of
individual links to ensure that their transmissions do not cause
interference on other links sharing the same wireless channel.

To this end, we define an additional set of real-valued decision
variables :math:`R` of size
:math:`N^2 \cdot M \cdot {C^\star_d}`. :math:`R(i,j,T,k)`
denotes the amount of bits of the data product of task :math:`T` that is
transmitted from agent :math:`i` to agent :math:`j` in time interval
:math:`k`.

We assume that, for each discrete time interval :math:`k` and for each
subset :math:`I\in \mathbb{I}\subset 2^{N^2}` of links subject to mutual
interference, the interfering links’ channel capacity :math:`r(I, k)`
(that is, the overall amount of bits that links in :math:`I` can
simultaneously transmit) is known. In order to avoid introducing an
exponential number of constraints, it is desirable to consider a modest
number of sets of interfering links. For instance, if all robots are
operating in close proximity and can interfere with each other, the
overall bandwidth of *all* links should be constrained to be smaller
than the capacity of the shared channel.

The fifth constraint in the formulation above is replaced by the
following equations:

.. math::

   \begin{aligned}
   & R(i,j,T,k) \leq r_{i,j}(k) C(i,j,T,k) \quad \forall  i,j \in[1,\ldots,N], T\in[1,\ldots,M], k\in[1,\ldots,T] \label{eq:MILP:boolean_bandwidth} \\
   %
   & D(i,T,k+1)\!-\!D(i,T,k) \quad \leq \sum_{\tau=1}^k\sum_{j=1}^N \frac{1}{{s\left({T}\right)}} R(j,i,T,k) + \sum_{\tau=1}^{{k-{\tau_{i}\left({T}\right)}}} X(i,T,k) \nonumber \\
   & \quad  \forall i\in[1,\ldots,N],  T\in[1,\ldots,M], k\in[1,\ldots,{C^\star_d}-1] \label{eq:MILP:learning_interference} \\
   %
   & \sum_{i,j \in {i}} \sum_{T\in[1,\ldots,M]} R(j,i,T,k) \leq r(i,k) \quad \forall i\in I, k\in[1,\ldots,T] \label{eq:MILP:channel_capacity}\end{aligned}


The first constraint ensures that the effective bit rate on a link is nonzero only if a
communication occurs on the link; The second constraint
models the process by which robots learn data products through
communication, closely following its counterpart in the previous formulation;
and the third constraint ensures that
the sum of all effective bit rates on interfering links does not exceed
the channel capacity.

Implementation
--------------
The function :class:`~mosaic_schedulers.schedulers.tv_milp.MOSAICSolver.JSONSolver` provides 
an easy interface to the task allocation solver.
For a full description of all available interfaces, see :ref:`tv_class_documentation`.

.. autoclass:: mosaic_schedulers.schedulers.tv_milp.MOSAICSolver.JSONSolver
   :members:
   :noindex:

References
----------

Joshua Vander Hook, Tiago Vaquero, Federico Rossi, Martina Troesch, Marc Sanchez Net, Joshua Schoolcraft, Jean-Pierre de la Croix, and Steve Chien, `"Mars On-Site Shared Analytics Information and Computing," <https://aaai.org/ojs/index.php/ICAPS/article/view/3556>`__ in Proceedings of the Twenty-Ninth International Conference on Automated Planning and Scheduling, vol. 29, no. 1, pp. 707-715, July 2019.