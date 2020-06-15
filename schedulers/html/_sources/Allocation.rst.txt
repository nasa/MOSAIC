.. _time_invariant:

Task allocation algorithm for robotic networks with periodic connectivity
=========================================================================


In this section, we propose a task allocation algorithm suitable for
multi-agent systems with *periodic* or *static* network connectivity.

A full description of the algorithm is available in `our paper <#references>`__, which also
presents a *distributed* implementation through a shared-world approach.

The goal of the algorithm is to allocate computational tasks to agents
while satisfying constraints on:

-  Logical dependencies between tasks: certain tasks require as input
   the outputs of other tasks;

-  Maximum latency of data products: the inputs to a task must be
   delivered to the agent performing it within a maximum latency from
   the moment they are computed;

-  Maximum computational load of computing agents;

-  Maximum throughout of communication links.

We cast the problem as a mixed-integer linear program (MILP) in a
network flow framework. Namely, data products transmitted from a task to
another are modeled as a network flow with an origin at the agent
performing the parent task and a destination at the the agent performing
the child task.

The key modeling assumption of the task allocation algorithm is that the
average available throughput on communication links is approximately
constant over a representative time period :math:`{T}`. With
this assumption, the problem is modeled through a time-invariant graph,
where nodes represent agents, edges represent communication links, and
binary decision variables capture the allocation of tasks to agents.

Problem Parameters
------------------

The problem is parameterized by the following properties:

-  :math:`{T}`, the time period of interest;

-  :math:`{\mathbb{T}}`, the set of computational tasks to be
   allocated in the time period of interest;

-  :math:`{R({{t}})}`, the set of tasks that
   require task :math:`t`\ ’s data products as inputs. Tasks
   :math:`\tau\in {R({{t}})}` are denoted as
   *children* of :math:`{t}`;

-  :math:`{d_{{t}}}`, the size of the data product
   of task :math:`t`, in bits;

-  :math:`{\mathbb{R}}\subseteq {\mathbb{T}}`, the
   set of *required* tasks that must be executed;

-  :math:`{\mathbb{O}}= {\mathbb{T}}\setminus {\mathbb{R}}`,
   the set of *optional* tasks;

-  :math:`{r({t})}`, the reward for planning the
   execution of optional tasks
   :math:`{t}\in{\mathbb{O}}`;

-  :math:`{\mathbb{N}}`, the set of available agents;

-  :math:`{\mathbb{E}}`, the set of pairs of agents
   :math:`(i, j)` that can directly communicate through a radio link;

-  :math:`{c_{{t}{i}}}`, the
   computational load (i.e., the average fraction of a CPU core,
   averaged over the time period :math:`T`) required to perform task
   :math:`t` once on agent :math:`i`;

-  :math:`{e_{{t}{i}}}`, the average
   power required to perform task :math:`t` once on node :math:`i` over
   time :math:`T`, in W;

-  :math:`{c^{\text{out}}_{ij}}`, the computational load
   (i.e., the fraction of a CPU core) required to encode one bit of
   information per second on the outbound stream on link :math:`(i,j)`;

-  :math:`{c^{\text{in}}_{ij}}`, the computational load to
   decode one bit of information per second from the inbound stream on
   :math:`(i,j)`;

-  :math:`{e^{\text{out}}_{ij}}`, the energy (in J/bit)
   required to encode and transmit one bit per second on the outbound
   stream on link :math:`(i,j)`;

-  :math:`{e^{\text{in}}_{ij}}`, the energy (in J/bit)
   required to decode one bit per second from the inbound stream on link
   :math:`(i,j)`;

-  :math:`{\overline{b}_{i,j}}`, the maximum available
   throughput (in bits/s) on the radio link
   :math:`(i,j) \in {\mathbb{E}}`;

-  :math:`{l_{i,j}}`, the light-speed latency of link
   :math:`(i,j)` (in s);

-  :math:`{\overline{c}_{{i}}}`, the maximum
   computational load of node :math:`i` (i.e., the number of CPU cores
   available at node :math:`{i}`);

-  :math:`{\overline{l}_{{t},\tau}}` is the
   maximum acceptable latency (in s) for the data products of task
   :math:`{t}` required by task :math:`\tau`.

Optimization Variables
----------------------

The optimization variables are:

-  :math:`X`, a matrix of Boolean variables of size
   :math:`|{\mathbb{N}}|` by
   :math:`|{\mathbb{T}}|`. :math:`X(i,t)` is set to 1 if task
   :math:`t` is assigned to agent :math:`i` and :math:`0` otherwise;

-  :math:`C`, a matrix of real-valued variables of size
   :math:`|{\mathbb{E}}|` by
   :math:`\sum_{t\in{\mathbb{T}}}|{R({t})}|`.
   :math:`C(i,j,t,\tau)` is the amount of data products of task
   :math:`t` (in bits per second) transmitted on link :math:`(i,j)` and
   that will be used as input by task :math:`\tau`;

-  :math:`B`, a matrix of real-valued variables of size
   :math:`|{\mathbb{E}}|` by
   :math:`|{\mathbb{T}}|`. :math:`B(i,j,t)` is the amount of
   bandwidth used by data products of task :math:`t` on link
   :math:`(i,j)` (irrespective of the destination).

The variables :math:`C` keep track of data products separately according
to the requiring child task; this enables the proposed model to to (i)
capture the average latency of the data products to multiple
destinations (as discussed `below <#latency>`__) and
(ii) represent logical dependencies between tasks by considering the
requiring task as an “information sink”. However, identical data
transmitted on the same link should not be duplicated, irrespective of
the destination. Variables :math:`B` capture the fact that, in a
practical implementation, different data flows :math:`C` containing the
same underlying data product are de-duplicated and transmitted once per
communication link.

Optimization Objective
----------------------

The optimization objective is the weighted sum of two terms capturing
(i) a reward for completing optional tasks and (ii) the power required
to solve the problem. The individual cost functions can be captured as
follows.

-  Maximize the sum of the rewards for completed optional tasks:

   .. math:: R_r =   \sum_{j\in{\mathbb{N}}} \sum_{t\in{\mathbb{T}}}  {r(t)} X(j,t) \label{eq:MILPgoal}

-  Minimize the power cost of the problem:

   .. math:: R_e = - \sum_{j\in{\mathbb{N}}} \sum_{t\in{\mathbb{T}}} {e_{tj}}X(j,t) + \sum_{(i,j)\in{\mathbb{E}}} \sum_{t\in{\mathbb{N}}} ({e^{\text{out}}_{ij}}+{e^{\text{in}}_{ij}}) B(i,j,t)

The optimization objective can then be expressed as
:math:`R = \alpha R_r + (1-\alpha) R_e,`, where :math:`\alpha` is the
relative weight of the two reward functions. [eq:MILP:costs]

Problem Formulation
-------------------

The problem can be posed as a mixed-integer linear program as follows.

.. math::

   \begin{aligned}
   &\max_{X, C, B}  R \label{eq:MILP:cost}\\
   &\text{subject to:}\nonumber \\
   & \sum_{j\in {\mathbb{N}}} X(j,t) = 1 \qquad \forall t \in {\mathbb{R}}\label{eq:MILP:requiredtasks} \\
   %
   & \sum_{j\in {\mathbb{N}}} X(j,t) \leq 1 \qquad \forall t \in {\mathbb{O}}\label{eq:MILP:optionaltasks} \\
   %
   & X(j,t) \frac{{d_{t}}}{{T}} + \sum_{i: (i,j)\in{\mathbb{E}}} C(i,j,t,\tau) \geq X(j,\tau) \frac{{d_{t}}}{{T}} + \sum_{k:(j,k\in{\mathbb{E}}} C(j,k,t\tau) \quad \forall j \in {\mathbb{N}}, t\in{\mathbb{T}}, \tau \in {R({t})} \label{eq:MILP:prereqs} \\
   %
   & B(i,j,t) \geq C(i,j,t,\tau) \quad \forall (i,j) \in {\mathbb{E}}, t\in {\mathbb{T}}, \tau\in {R({t})} \label{eq:MILP:multiplex}\\
   %
   & \sum_{t\in{\mathbb{T}}} {c_{tj}} X(j,t) + \sum_{i: (i,j) \in{\mathbb{E}}}\sum_{t\in{\mathbb{T}}} {c^{\text{in}}_{ij}}B(i,j,t) \nonumber + \sum_{k: (j,k) \in{\mathbb{E}}}\sum_{t\in{\mathbb{T}}} {c^{\text{out}}_{jk}} B(j,k,t) \leq {\overline{c}_{j}}  \qquad \forall j\in{\mathbb{N}}\label{eq:MILP:computation}\\
   %
   &\sum_{t\in\ {\mathbb{T}}} B(i,j,t) \leq {\overline{b}_{i,j}} \qquad \forall (i,j) \in {\mathbb{E}}\label{eq:MILP:bandwidth}\\ %Note: we assume we transmit at the max bandwidth whenever available.
   %
   & \sum_{(i, j) \in{\mathbb{E}}} \left({l_{i,j}} + \frac{{d_{t}}}{{\overline{b}_{i,j}}}\right) C(i,j,t, \tau) \frac{{T}}{{d_{t}}}    \leq {\overline{l}_{t,\tau}} \quad \forall t\in {\mathbb{T}}, \tau   \in {R({t})}
   \label{eq:MILP:latency}\end{aligned}

The first constraint ensures
that all required tasks are allocated.
The second constraint ensures
that optional tasks are allocated at most once.
The third constraint  (discussed 
`below <#prerequisites>`__) ensures that nodes
only execute a task if they have access to the data products of its
pre-requisites.
The fourth constraint 
ensures that the variable :math:`D` is an upper bound on the data
throughput used by data products of task :math:`t`.
The fifth constraint enforces that
the computational capacity of each node is not exceeded.
The sixth constraint ensures that the
transmissions on a link do not exceed the available bandwidth.
Finally,
the seventh constraint (discussed `below <#latency>`__) guarantees that data products are
delivered to dependent tasks within a maximum acceptable latency.

.. _prerequisites:

Prerequisites
^^^^^^^^^^^^^

The prerequisites constraint ensures that a copy of
the data product of task :math:`t` is created at the node where task
:math:`t` is performed, and it is consumed at the node where task
:math:`\tau` is performed. At other nodes, information can be relayed,
but neither created nor destroyed.

.. _latency:

Latency
^^^^^^^

The latency constraint ensures that the
*average* latency of data products of task :math:`t` used by task
:math:`\tau` is smaller than :math:`\overline{l}_{t,\tau}`. The latency
for the data products of task :math:`t` destined for task :math:`\tau`
on a *given* path :math:`p=\{(u,v), (v, w), \ldots, \}` is
:math:`\sum_{(i,j) \in p} ({l_{i,j}} + {d_{t}}/{\overline{b}_{i,j}})`.
If data products are sent on multiple paths :math:`p_1, \ldots, p_m`,
with path :math:`p_k` carrying a fraction :math:`f_k` of the data
product, the average latency is:

.. math::

   \begin{aligned}
    l_{t,\tau} & = \sum_{k=1}^m f_k \sum_{(i,j)\in p_i} ({l_{i,j}} + {d_{t}}/{\overline{b}_{i,j}}) = \sum_{(i,j) \in {\mathbb{E}}} ({l_{i,j}} + {d_{t}}/{\overline{b}_{i,j}}) \sum_{k=1}^m 1_{(i,j)\in p_k} f_k\end{aligned}

The flow :math:`C(i,j,t,\tau)` captures the sum of path flows on all
paths carrying data products of :math:`t` for task :math:`\tau`. The
overall amount of flow on all paths is the size of the data product
:math:`d_t` divided by the time period :math:`T`. Accordingly, the
average latency can be rewritten as

.. math:: l_{t,\tau} = \sum_{(i,j) \in {\mathbb{E}}} ({l_{i,j}} + {d_{t}}/{\overline{b}_{i,j}}) C(i,j,t,\tau)\frac{{T}}{{d_{t}}}

in accordance with the latency constraint.

Implementation
--------------
The function :class:`~mosaic_schedulers.schedulers.ti_milp.MOSAICTISolver.JSONSolver` provides 
an easy interface to the task allocation solver.
For a full description of other available interfaces, see :ref:`ti_class_documentation`.

.. autoclass:: mosaic_schedulers.schedulers.ti_milp.MOSAICTISolver.JSONSolver
   :members:
   :noindex:

References
----------

Federico Rossi\*, Tiago Stegun Vaquero\*, Marc Sanchez Net, Maíra Saboia da Silva, and Joshua Vander Hook, `"The Pluggable Distributed Resource Allocator (PDRA): a Middleware for Distributed Computing in Mobile Robotic Networks," <https://arxiv.org/abs/2003.13813>`__ under review.