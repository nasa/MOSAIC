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

"""
A modified version of the HEFT scheduler that accounts for communication time
between the agents (but not for optional tasks).
The key modifications with respect to the HEFT scheduler are:
- Computation tasks take time and tie up agent resources. They are scheduled
  according to the insertion-based scheduler; longer communications are 
  scheduled first.
- The agent providing the earliest _finish_ time (as opposed to the earliest
  start time) is assigned to the task.
"""




import networkx as nx
import numpy as np
import json
import sys
import copy
from mosaic_schedulers.common.utilities.contact_plan_handler import compute_time_invariant_bandwidth
class Scheduler():
    def __init__(self,
                 json_problem_description):

        self.is_labeled = False
        self.is_upward_ranked = False
        self.is_scheduled = False

        if type(json_problem_description) is dict:
            json_input = json_problem_description
        else:
            json_input = json.loads(json_problem_description)

        # Tasks
        optional_tasks = json_input["Tasks"]["OptionalTasks"]
        # TaskReward=json_input["Tasks"]["TaskReward"]
        self.products_size = json_input["Tasks"]["ProductsSize"]
        dependency_list_disjunctive = json_input["Tasks"]["DependencyList"]
        # IncompatibleTasks=json_input["Tasks"]["IncompatibleTasks"]
        tasks_list = list(optional_tasks.keys())
        if len(tasks_list) == 0:
            raise ValueError("ERROR: no tasks")

        dependency_list = {}
        for task in dependency_list_disjunctive.keys():
            if dependency_list_disjunctive[task]:
                dependency_list[task] = []
                for dep in dependency_list_disjunctive[task]:
                    if dep:
                        assert len(
                            dep) == 1, 'Disjunctive prerequirements are not supported'
                        dependency_list[task] += [dep[0]]

        # Agent capabilities
        self.computation_time = json_input["AgentCapabilities"]["ComputationTime"]
        # ComputationLoad=json_input["AgentCapabilities"]["ComputationLoad"]
        # InitialInformation=json_input["AgentCapabilities"]["InitialInformation"]
        # EnergyCost=json_input["AgentCapabilities"]["EnergyCost"]
        self.agents_list = list(self.computation_time[tasks_list[0]].keys())
        if len(self.agents_list) == 0:
            raise ValueError("ERROR: no agents")

        # # Timing
        thor_s = json_input["Time"]["Thor"]
        # time_step = json_input["Time"]["TimeStep"]
        # thor = int(float(thor_s) / float(time_step))
        self.allowed_agents = {}
        for task in tasks_list:
            self.allowed_agents[task] = []
            for agent in self.agents_list:
                if self.computation_time[task][agent] < thor_s:
                    self.allowed_agents[task].append(agent)

        # Options
        #Options = json_input["Options"]

        # Communication bandwidth
        tv_bandwidth = json_input["CommunicationNetwork"]
        od_pairs = []
        for link in tv_bandwidth:
            od_pairs += [(link["origin"], link["destination"])]
        assert len(set(od_pairs)) == len(
            od_pairs), "ERROR: some links are repeated (is the problem time-varying?)"

        agent_names = json_input["AgentCapabilities"]["MaxComputationLoad"].keys(
        )

        self.comm_windows_bandwidth, _, _ = compute_time_invariant_bandwidth(
            agents_list=agent_names,
            TVBandwidth=tv_bandwidth,
            Thor=thor_s
        )
        # self.comm_windows_bandwidth = {}
        # #comm_energy_cost = {}
        # # Initialize
        # for agent1 in agent_names:
        #     self.comm_windows_bandwidth[agent1] = {}
        #     #comm_energy_cost[agent1] = {}
        #     for agent2 in agent_names:
        #         self.comm_windows_bandwidth[agent1][agent2] = 0.
        #         #comm_energy_cost[agent1][agent2] = 0.
        # # Fill in
        # for link in tv_bandwidth:
        #     self.comm_windows_bandwidth[link["origin"]
        #                                 ][link["destination"]] = link["bandwidth"]
        #     #comm_energy_cost[agent1][agent2] = link["energy_cost"]

        software_network = nx.DiGraph()

        all_tasks = set(list(self.allowed_agents.keys()) +
                        list(dependency_list.keys()))
        for task in all_tasks:
            if len(self.allowed_agents[task]) > 0:
                software_network.add_node(task)

        # This misses the case where a task has no dependencies and is nobody's dependency
        # for task in dependency_list.keys():
        #     if len(self.allowed_agents[task]) > 0:
        #         software_network.add_node(task)
        # software_network.add_nodes_from(dependency_list.keys())
        for task in dependency_list.keys():
            if len(self.allowed_agents[task]) > 0:
                for dep in dependency_list[task]:
                    software_network.add_edge(dep, task)

        self.software_network = software_network

    # Label the software_network with relevant computation times (on nodes) and comm time (on edges)
    def label_software_network(self):
        for task in self.software_network.nodes():
            self.software_network.node[task]["processing_time"] = 0.
            for agent in self.allowed_agents[task]:
                self.software_network.node[task]["processing_time"] += self.computation_time[task][agent]
            self.software_network.node[task]["processing_time"] /= float(
                len(self.allowed_agents[task]))

        for pred, succ in self.software_network.edges():
            self.software_network.edges[pred, succ]["communication_time"] = 0.
            valid_links = 0
            for agent_pred in self.allowed_agents[pred]:
                for agent_succ in self.allowed_agents[succ]:
                    if agent_pred == agent_succ:
                        valid_links += 1
                    elif self.comm_windows_bandwidth[agent_pred][agent_succ] > 0:
                        self.software_network.edges[pred, succ]["communication_time"] += self.products_size[pred] / \
                            self.comm_windows_bandwidth[agent_pred][agent_succ]
                        valid_links += 1
            if valid_links > 0:
                self.software_network.edges[pred,
                                            succ]["communication_time"] /= valid_links
            else:
                # If there are no valid links, the comm time should stay at zero.
                assert self.software_network.edges[pred,
                                                   succ]["communication_time"] == 0.

        self.is_labeled = True

    def compute_upward_rank(self):
        if self.is_labeled is False:
            self.label_software_network()

        def recursive_upward_rank(node, depth):
            if depth > len(self.software_network):
                raise ValueError(
                    "Recursion depth exceeded! Make sure that the software network is acyclic.")
            if len(list(self.software_network.successors(node))) == 0:
                self.software_network.nodes[node]["upward_rank"] = self.software_network.nodes[node]["processing_time"]
            elif "upward_rank" in self.software_network.nodes[node].keys():
                pass
            else:
                max_successor_rank = 0
                for successor in self.software_network.successors(node):
                    # Recurse
                    max_successor_rank = max(max_successor_rank, self.software_network.nodes[node]["processing_time"]+recursive_upward_rank(
                        successor, depth+1) + self.software_network.edges[node, successor]["communication_time"])
                # Memoize
                self.software_network.nodes[node]["upward_rank"] = max_successor_rank
            # Return for recursion
            return self.software_network.nodes[node]["upward_rank"]
        # To ensure full coverage, we do the dumb thing and call recursive_upward_rank on every node.
        # Note that the algorithm is actually DP: if we already visited a node, we do not recurse on it.
        for node in self.software_network:
            recursive_upward_rank(node, 0)
        self.is_upward_ranked = True
        # Sort the tasks according to their rank
        ranked_node_dict = nx.get_node_attributes(
            self.software_network, "upward_rank")
        self.ranked_nodes = sorted(
            ranked_node_dict, key=ranked_node_dict.__getitem__, reverse=True)

    # Schedule the software_network
    def schedule(self):
        if self.is_labeled is False:
            self.label_software_network()
        if self.is_upward_ranked is False:
            self.compute_upward_rank()
        # Schedule each task
        # Keep a running list of the tasks assigned to each processor, initially empty
        self.TaskAllocation = {}

        schedule = {}
        tasks_by_agent_detailed = {}
        for agent in self.agents_list:
            tasks_by_agent_detailed[agent] = []

        # For every task, in ranked order
        for task in self.ranked_nodes:
            scheduled_time = sys.float_info.max
            est_by_agent = {}
            eft_by_agent = {}
            all_tentative_tasks_by_agent = {}
            # For every feasible agent
            for agent in self.allowed_agents[task]:
                est = 0
                # How long to compute
                processing_time = self.computation_time[task][agent]
                # Earliest we can hear from a predecessor
                tentative_tasks_by_agent = copy.deepcopy(
                    tasks_by_agent_detailed)  # Copy the list
                if len(list(self.software_network.predecessors(task))) > 0:
                    pred_comm_duration = {}
                    # What is the earliest we can schedule a task, if communication did not take any CPU power?
                    for predecessor in self.software_network.predecessors(task):
                        # Below we use the fact that predecessors of a task have already been scheduled,
                        #  since we are proceeding according to upward rank
                        if schedule[predecessor]['agent'] == agent:
                            predecessor_comm_time = 0   # Within-agent communication is instant
                        elif self.comm_windows_bandwidth[schedule[predecessor]['agent']][agent] == 0.:
                            predecessor_comm_time = sys.float_info.max  # Can't communicate at all
                        else:
                            predecessor_comm_time = self.products_size[predecessor] / \
                                self.comm_windows_bandwidth[schedule[predecessor]
                                                            ['agent']][agent]
                        pred_comm_duration[predecessor] = predecessor_comm_time
                        est = max(
                            est, schedule[predecessor]['finish_time'] + predecessor_comm_time)

                    # Now let's see when we can _actually_ communicate by trying to schedule communication tasks
                    sorted_comm_tasks = sorted(
                        pred_comm_duration, key=pred_comm_duration.__getitem__, reverse=True)
                    # Make a copy of the existing schedule. For every agent, we will tentatively schedule a bunch of communication tasks.
                    # Note that we do NOT copy schedule. We do not rely on it to compute when an agent is free, only for a quick look-up as to where/when a task is scheduled
                    for predecessor in sorted_comm_tasks:
                        # Schedule the communication tasks one by one
                        pred_agent = schedule[predecessor]['agent']
                        if pred_agent != agent and self.comm_windows_bandwidth[pred_agent][agent] >= 0.:
                            # When are both the predecessor's agent and the ego agent free?
                            comm_windows = self.blend_schedules(
                                {pred_agent: tentative_tasks_by_agent[pred_agent], agent: tentative_tasks_by_agent[agent]})
                            # Now schedule the communication task
                            comm_to_schedule = {'earliest_start_time': schedule[predecessor]['finish_time'],
                                                'processing_time': pred_comm_duration[predecessor], 'agent': None, 'task': 'transfer_data'}
                            scheduled_comm = self.insertion_based_heuristic(
                                comm_to_schedule, comm_windows)
                            # Store the resulting task
                            scheduled_comm_sender = scheduled_comm.copy()
                            scheduled_comm_sender['agent'] = pred_agent
                            scheduled_comm_sender['params'] = {
                                'transmitter': pred_agent,
                                'receiver': agent,
                                'data_type': task
                            }
                            tentative_tasks_by_agent[pred_agent].append(
                                scheduled_comm_sender)

                            tentative_tasks_by_agent[pred_agent].sort(
                                key=lambda x: x['start_time'])
                            scheduled_comm_receiver = scheduled_comm_sender.copy()
                            scheduled_comm_receiver['agent'] = agent
                            tentative_tasks_by_agent[agent].append(
                                scheduled_comm_receiver)
                            tentative_tasks_by_agent[agent].sort(
                                key=lambda x: x['start_time'])
                            # ... and update the tentative start time
                            est = max(
                                est, scheduled_comm_receiver['finish_time'])

                # Now let's look for a gap for the task itself
                task_to_schedule = {'earliest_start_time': est,
                                    'processing_time': processing_time, 'agent': agent, 'task': task}
                tentative_scheduled_task = self.insertion_based_heuristic(
                    task_to_schedule, tasks_by_agent_detailed[agent])
                est_by_agent[agent] = tentative_scheduled_task['start_time']
                eft_by_agent[agent] = tentative_scheduled_task['finish_time']
                all_tentative_tasks_by_agent[agent] = tentative_tasks_by_agent

            ranked_agents = sorted(eft_by_agent, key=eft_by_agent.__getitem__)
            scheduled_agent = ranked_agents[0]
            # Add the prerequisite comm tasks, overwriting the previous allocation
            tasks_by_agent_detailed = all_tentative_tasks_by_agent[scheduled_agent]
            # Add the task itself
            scheduled_time = est_by_agent[scheduled_agent]
            duration = self.computation_time[task][scheduled_agent]
            task_description = {'task': task, 'start_time': scheduled_time,
                                'finish_time': scheduled_time+duration, 'agent': scheduled_agent}
            schedule[task] = task_description
            tasks_by_agent_detailed[scheduled_agent].append(task_description)
            # Sorting below: Expensive? Yes. Unnecessary? Yes (we know where we found a spot in insertion_based). To be improved if it turns out to be a significant bottleneck.
            tasks_by_agent_detailed[scheduled_agent].sort(
                key=lambda x: x['start_time'])

        self.task_schedule = schedule
        self.tasks_by_agent_detailed = tasks_by_agent_detailed

        self.is_scheduled = True

        #self.json_schedule = self.jsonify_schedule_no_comms(self.task_schedule)
        self.json_schedule = self.jsonify_schedule(
            self.tasks_by_agent_detailed)
        return self.json_schedule

    def insertion_based_heuristic(self, task, current_agent_timeline):
        est = task['earliest_start_time']
        processing_time = task['processing_time']
        for existing_task in current_agent_timeline:
            if existing_task['start_time'] > est + processing_time:
                # We fit in this gap!
                break
            else:
                est = max(est, existing_task['finish_time'])
        return {'start_time': est, 'finish_time': est+processing_time, 'agent': task['agent'], 'task': task['task']}

    def blend_schedules(self, tasks_by_agent_detailed):
        '''
        Takes in multiple schedules and returns a "merged" one
        that blocks time when at least one schedule is busy.
        Useful to find gaps when communication is possible.
        '''
        events = []
        for agent in tasks_by_agent_detailed.keys():
            for task in tasks_by_agent_detailed[agent]:
                events.append({'time': task["start_time"], 'type': 'start'})
                events.append({'time': task["finish_time"], 'type': 'end'})
        events.sort(key=lambda x: x['time'])

        blended_events = []
        active_tasks = 0
        for event in events:
            if event['type'] is 'start':
                active_tasks += 1
                if active_tasks == 1:  # if we just started
                    blended_event = {}
                    blended_event['start_time'] = event['time']
            else:
                active_tasks -= 1
                if active_tasks == 0:  # If we just finished
                    blended_event['finish_time'] = event['time']
                    blended_events.append(blended_event)
                    del blended_event
        return blended_events

    def jsonify_schedule_no_comms(self, task_schedule):
        json_schedule = []
        for task_name, task in task_schedule.items():
            task_dict = {
                "duration": task["finish_time"] - task["start_time"],
                "start_time": task["start_time"],
                "params": {
                    "agent": task["agent"]
                },
                "id": task_name,
                "name": task_name,
            }
            json_schedule.append(task_dict)
        return json.dumps({"tasks": json_schedule})

    def jsonify_schedule(self, tasks_by_agent):
        json_schedule = []
        for agent, tasks_list in tasks_by_agent.items():
            for task in tasks_list:
                # for task_name, task in task_schedule.iteritems():
                task_dict = {
                    "duration": task["finish_time"] - task["start_time"],
                    "start_time": task["start_time"],
                    "params": {
                        "agent": task["agent"]
                    },
                    "id": task["task"],
                    "name": task["task"],
                }
                if task_dict["name"] == "transfer_data":
                    task_dict["params"]["transmitter"] = task["params"]["transmitter"]
                    task_dict["params"]["receiver"] = task["params"]["receiver"]
                    task_dict["params"]["data_type"] = task["params"]["data_type"]
                    task_dict["params"]["bandwidth"] = self.comm_windows_bandwidth[task["params"]
                                                                                   ["transmitter"]][task["params"]["receiver"]]
                    task_dict["params"]["agent"] = task["params"]["transmitter"]
                json_schedule.append(task_dict)
        return json.dumps({"tasks": json_schedule})


if __name__ == "__main__":
    import mosaic_schedulers.common.examples.PUFFER.ScenarioGenerator as PUFFERGenerator
    import mosaic_schedulers.common.examples.PUFFER.MILPProblemGenerator as MILPProblemGenerator
    from mosaic_schedulers.common.plotting import MOSAICplotter
    Scenario = PUFFERGenerator.create_scenario(
        num_puffers=2, base_station=True, science_zone_probability=1., science_samples_per_puffer=1)
    Problem = MILPProblemGenerator.MOSAICProblem(Scenario)
    JSON_input = Problem.createJSON()
    heft_scheduler = Scheduler(JSON_input)
    heft_scheduler.label_software_network()
    heft_scheduler.compute_upward_rank()
    json_schedule = heft_scheduler.schedule()
    MOSAICplotter.SchedulePlotter(json_schedule)
