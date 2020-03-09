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

from mosaic_schedulers.schedulers import ti_milp, heft
import json
import copy


class Scheduler():
    def __init__(self, json_problem_description, solver='GLPK', TimeLimit=None, Verbose=False):
        self.json_problem_description = json_problem_description
        self.relay_duration = 0.001
        self.solver = solver
        self.TimeLimit = TimeLimit
        self.verbose = Verbose

    def verbprint(self, strpr):
        if self.verbose:
            print(strpr)

    def schedule_ti_milp(self):
        self.verbprint("Solving TI MILP")
        # Create and solve a ti-milp problem
        ti_milp_scheduler = ti_milp.JSONSolver(
            self.json_problem_description, solver=self.solver, TimeLimit=self.TimeLimit)
        ti_schedule = ti_milp_scheduler.schedule()
        self.ti_schedule = json.loads(ti_schedule)

    def prepare_heft(self):
        self.verbprint("Preparing HEFT")
        # An easy way to look for tasks that will be useful later
        task_list = {}
        comm_list = {}
        for task in self.ti_schedule["tasks"]:
            if task["name"] == "transfer_data":
                if task["params"]["data_type"] not in comm_list.keys():
                    comm_list[task["params"]["data_type"]] = []
                comm_list[task["params"]["data_type"]].append(task)
            else:
                task_list[task["name"]] = task

        # Turn the dependency list into something we can easily use
        self.verbprint(
            "Turn the dependency list into something we can easily use")
        json_input = json.loads(self.json_problem_description)
        dependency_list_disjunctive = json_input["Tasks"]["DependencyList"]
        dependency_list = {}
        for task in dependency_list_disjunctive.keys():
            if dependency_list_disjunctive[task]:
                dependency_list[task] = []
                for dep in dependency_list_disjunctive[task]:
                    if dep:
                        assert len(
                            dep) == 1, 'Disjunctive prerequirements are not supported'
                        dependency_list[task] += [dep[0]]

        # Create a copy of the relevant inputs for HEFT
        self.verbprint("Create a copy of the relevant inputs for HEFT")
        heft_optional_tasks = copy.deepcopy(
            json_input["Tasks"]["OptionalTasks"])
        heft_products_size = copy.deepcopy(json_input["Tasks"]["ProductsSize"])
        heft_dependency_list = copy.deepcopy(dependency_list)
        heft_computation_time = copy.deepcopy(
            json_input["AgentCapabilities"]["ComputationTime"])
        thor = json_input["Time"]["Thor"]

        heft_agents = json_input["AgentCapabilities"]["MaxComputationLoad"].keys(
        )

        # Tweak the HEFT instance to pin tasks to agents
        self.verbprint("Tweak the HEFT instance to pin tasks to agents")
        #  First, remove optional tasks that we will not perform
        heft_optional_tasks_keys = list(heft_optional_tasks.keys())
        for task in heft_optional_tasks_keys:
            if task not in task_list:
                # ti-milp tells us not to perform the task. So it's optional, right?
                assert json_input["Tasks"]["OptionalTasks"][task] == True
                heft_optional_tasks.pop(task)
                heft_products_size.pop(task)
                heft_dependency_list.pop(task)
                heft_computation_time.pop(task)
                # Keep it in the computation time - it shouldn't hurt
            else:
                heft_optional_tasks[task] = False

        # Next, pin tasks to agents
        self.verbprint("Next, pin tasks to agents")
        for task in task_list:
            for agent in heft_agents:
                if agent != task_list[task]["params"]["agent"]:
                    heft_computation_time[task][agent] = thor+1

        # Next, look for cases where information is relayed across multiple agents. In that case, create an intermediate "relay" task.
        self.verbprint(
            "Next, look for cases where information is relayed across multiple agents.")
        for task, dependencies in dependency_list.items():
            if task in task_list.keys():  # Do we actually perform this task?
                for pred in dependencies:  # For every dependency
                    assert pred in task_list.keys()  # Of course we also perform the dependency
                    task_agent = task_list[task]["params"]["agent"]
                    task_pred_agent = task_list[pred]["params"]["agent"]
                    direct_comm = False
                    if task_agent == task_pred_agent:  # If the task and the parent are on the same node, do nothing
                        direct_comm = True
                    else:
                        # Go through the communications containing the prerequisite
                        for comm in comm_list[pred]:
                            # If there is a communication with the relevant topic between them
                            if ((comm["params"]["transmitter"] == task_pred_agent) and (comm["params"]["receiver"] == task_agent)):
                                direct_comm = True
                                break   # We have a direct communication - HEFT will figure out how to schedule it
                    if direct_comm == False:  # If neither of the above is true, we have a chain!
                        self.verbprint("We have a chain!")
                        chain_node = task_agent
                        chain = [chain_node]
                        while chain_node != task_pred_agent:  # We walk backwards across communications, starting from the receiver, until we arrive at the transmitter
                            for comm in comm_list[pred]:
                                # This is the relevant communication
                                if comm["params"]["receiver"] == chain_node:
                                    # Store the entry in the chain
                                    # The transmitter must have received the communication from someone...unless it did the computation itself
                                    chain_node = comm["params"]["transmitter"]
                                    chain.append(chain_node)
                        assert len(chain) > 2  # A chain has at least one relay
                        chain_predecessor = pred
                        relay_number = 0
                        for node in chain[1:-1]:
                            relay_name = 'relay{}:'.format(relay_number)+pred
                            relay_number += 1
                            # Add a dependency on pred
                            heft_dependency_list[relay_name] = [
                                chain_predecessor]
                            # Create all the needed entries for the relay task
                            heft_optional_tasks[relay_name] = False
                            # products size is the same as the original one
                            heft_products_size[relay_name] = heft_products_size[pred]
                            # only node can relay it in zero time, everyone else takes infty
                            heft_computation_time[relay_name] = {}
                            for agent in heft_agents:
                                if agent == node:
                                    heft_computation_time[relay_name][agent] = self.relay_duration
                                else:
                                    heft_computation_time[relay_name][agent] = thor + 1
                            # Point to the predecessor
                            chain_predecessor = relay_name

                        # Last task: add a dependency on the last relay
                        heft_dependency_list[task].remove(pred)
                        heft_dependency_list[task].append(chain_predecessor)

        # Rebuild a disjunctive dependency list
        self.verbprint("Rebuild a disjunctive dependency list")
        heft_disjunctive_dependency_list = {}
        for key, predecessors in heft_dependency_list.items():
            heft_disjunctive_dependency_list[key] = list(map(
                lambda x: [x], predecessors))

        # Now build the HEFT problem description
        self.verbprint("Now build the HEFT problem description")
        heft_input = {}
        heft_input["Tasks"] = {}
        heft_input["Tasks"]["OptionalTasks"] = heft_optional_tasks
        heft_input["Tasks"]["ProductsSize"] = heft_products_size
        heft_input["Tasks"]["DependencyList"] = heft_disjunctive_dependency_list
        heft_input["AgentCapabilities"] = {}
        heft_input["AgentCapabilities"]["ComputationTime"] = heft_computation_time
        heft_input["AgentCapabilities"]["MaxComputationLoad"] = json_input["AgentCapabilities"]["MaxComputationLoad"]
        heft_input["Time"] = json_input["Time"]
        heft_input["CommunicationNetwork"] = json_input["CommunicationNetwork"]
        self.heft_input = heft_input

    def schedule_heft(self):
        self.verbprint("Scheduling HEFT")
        # Create a HEFT problem instance
        heft_scheduler = heft.Scheduler(json.dumps(self.heft_input))
        heft_schedule = heft_scheduler.schedule()
        self.heft_schedule = heft_schedule
        return self.heft_schedule

    def clean_up_heft(self):
        self.verbprint("Cleaning up HEFT")
        # Remove relay tasks
        _tasks = json.loads(self.heft_schedule)['tasks']
        tasks_to_keep = []
        for task in _tasks:
            if task['id'].find('relay') == -1:
                tasks_to_keep.append(task)

        self.heft_schedule = json.dumps({'tasks': tasks_to_keep})
        return self.heft_schedule

    def schedule(self):
        self.verbprint("Solving MILP-HEFT problem")
        self.schedule_ti_milp()
        if not self.ti_schedule["tasks"]:
            # If there are no tasks to schedule, skip HEFT
            return json.dumps(self.ti_schedule)
        self.prepare_heft()
        # If TI has decided to skip all tasks, skip HEFT
        if not self.heft_input["Tasks"]["OptionalTasks"]:
            return json.dumps(self.ti_schedule)
        self.schedule_heft()
        self.clean_up_heft()
        return self.heft_schedule


if __name__ == "__main__":
    import networkx as nx
    import matplotlib.pyplot as plt
    import mosaic_schedulers.common.examples.PUFFER.ScenarioGenerator as PUFFERGenerator
    import mosaic_schedulers.common.examples.PUFFER.MILPProblemGenerator as MILPProblemGenerator
    from mosaic_schedulers.common.plotting import MOSAICplotter
    # Create a scenario
    Scenario = PUFFERGenerator.create_scenario(
        num_puffers=3, base_station=True, science_zone_probability=1., science_samples_per_puffer=1)
    Problem = MILPProblemGenerator.MOSAICProblem(Scenario)
    JSON_input = Problem.createJSON()
    Colors = Problem.getTaskColors()
    # Solve it
    ti_milp_heft_scheduler = Scheduler(JSON_input)
    heft_schedule = ti_milp_heft_scheduler.schedule()
    # ti_milp_heft_scheduler.prepare_heft()

    #heft_scheduler = heft.Scheduler(json.dumps(ti_milp_heft_scheduler.heft_input))

    #heft_schedule = heft_scheduler.schedule()
    MOSAICplotter.SchedulePlotter(heft_schedule, TaskColors=Colors)
