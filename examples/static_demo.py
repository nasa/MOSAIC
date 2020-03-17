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
import state_reader
import problem_generator
import json


def create_problem_input(science_nodes_number=1, assist_nodes_number=1, inter_puffer_bw=0.01, base_station_bw=0.01, assembly_chain_bw_hop1=0.01, assembly_chain_bw_hop2=0.01, time_horizon=30, agent='puffer1',):
    science_puffers = ['puffer{}'.format(pufferno)
                       for pufferno in range(science_nodes_number)]
    assist_puffers = ['puffer{}'.format(
        pufferno+science_nodes_number) for pufferno in range(assist_nodes_number)]

    inter_agent_links = {(agent1, agent2): {'bandwidth': inter_puffer_bw, 'time': 832.9}
                         for agent1 in science_puffers+assist_puffers for agent2 in science_puffers+assist_puffers}

    base_station_links = {(agent1, 'base_station'): {
        'bandwidth': base_station_bw, 'time': 832.9} for agent1 in science_puffers+assist_puffers}

    base_station_links.update({('base_station', agent1): {
                              'bandwidth': base_station_bw, 'time': 832.9} for agent1 in science_puffers+assist_puffers})

    assembly_chain_links = {key: {'bandwidth': max(
        assembly_chain_bw_hop1, inter_puffer_bw), 'time': 832.9} for key in zip(science_puffers, assist_puffers)}
    assembly_chain_links.update({(agent1, 'base_station'): {'bandwidth': max(
        assembly_chain_bw_hop2, base_station_bw), 'time': 832.9} for agent1 in assist_puffers})

    inter_agent_links.update(base_station_links)
    inter_agent_links.update(assembly_chain_links)

    nodes = {agent: {'in_science_zone': True,  'battery_discrete_level': 3, }
             for agent in science_puffers}
    nodes.update({agent: {'in_science_zone': False,
                          'battery_discrete_level': 4, } for agent in assist_puffers})
    nodes.update({'base_station': {'in_science_zone': False,
                                   'battery_discrete_level': 4, 'is_base_station': True}, })

    problem_state = {
        'time_horizon': time_horizon,
        'Agent': agent,
        'link_state': inter_agent_links,
        'nodes': nodes,
    }

    communication_network = []
    for link in problem_state['link_state'].keys():
        props = problem_state['link_state'][link]
        communication_network.append(
            {"bandwidth": props['bandwidth'],
                "destination": link[1],
                "energy_cost": 1.0,
                "latency": 0.001,
                "origin": link[0],
                "time_end": 1.7976931348623157e+308,
                "time_start": 0.,
                "info_time": 0.}
        )

    return problem_state


def demo_input(demo_type="assembly_line", network_size='large', time_horizon=30):
    """
    Creates an input file for the ICAPS 2019 static demo.
    Takes two inputs, "demo_type" and "network_size".
    `demo_type` can assume four values:
    - `selfish`: agents cannot communicate.
    - `node_assist`: nearby nodes can help
    - `base_station_assist`: the base station can help
    - `assembly_line`: nodes not in the science zone assist both as relays and as computational assists.
    `network_size` can take two values, `large` (eight PUFFERs) and `small` (two PUFFERs).
    """

    demo_types = {
        "selfish": {
            'inter_puffer_bw': 0.01,
            'base_station_bw': 0.01,
            'assembly_chain_bw_hop1': 0.01,
            'assembly_chain_bw_hop2': 0.01,
        },
        "node_assist": {
            'inter_puffer_bw': 2.0,
            'base_station_bw': 0.01,
            'assembly_chain_bw_hop1': 0.01,
            'assembly_chain_bw_hop2': 0.01,
        },
        "base_station_assist": {
            'inter_puffer_bw': 0.01,
            'base_station_bw': 3.0,
            'assembly_chain_bw_hop1': 0.01,
            'assembly_chain_bw_hop2': 0.01,
        },
        "assembly_line": {
            'inter_puffer_bw': 0.01,
            'base_station_bw': 0.01,
            'assembly_chain_bw_hop1': 5.5,
            'assembly_chain_bw_hop2': 0.5,
        },
    }

    agents_number = {
        "large": {
            "science_nodes_number": 4,
            "assist_nodes_number": 4,
        },
        "small": {
            "science_nodes_number": 1,
            "assist_nodes_number": 1,
        },
    }

    if demo_type not in demo_types.keys():
        print("WARNING: demo_type {} is invalid. Allowed types are {}. Defaulting to {}".format(
            demo_type, demo_types.keys(), demo_types.keys()[-1]))
        demo_type = demo_types.keys()[-1]

    if network_size not in agents_number.keys():
        print("WARNING: network_size {} is invalid. Allowed types are {}. Defaulting to {}".format(
            network_size, agents_number.keys(), agents_number.keys()[-1]))
        network_size = agents_number.keys()[-1]

    problem_inputs = {}
    problem_inputs.update(agents_number[network_size])
    problem_inputs.update(demo_types[demo_type])
    problem_inputs.update({'time_horizon': time_horizon})
    return create_problem_input(**problem_inputs)


def create_comm_network(problem_state):
    communication_network = []
    for link in problem_state['link_state'].keys():
        props = problem_state['link_state'][link]
        communication_network.append(
            {
                "bandwidth": props['bandwidth'],
                "destination": link[1],
                "energy_cost": 1.0,
                "latency": 0.001,
                "origin": link[0],
                "time_end": 1.7976931348623157e+308,
                "time_start": 0.,
                "info_time": 0.,
            }
        )
    return communication_network


def create_json_description(problem_state):
    communication_network = create_comm_network(problem_state)

    # Problem definition in common format
    # agent_states = problem_state["nodes"]
    # agent = problem_state["Agent"]

    ScenarioParams = state_reader.create_scenario(
        Agent=problem_state["Agent"],
        agent_states=json.dumps(problem_state["nodes"]),
        network_state=json.dumps(communication_network),
        current_time=1,
        time_horizon=problem_state["time_horizon"]
    )

    # Problem definition in MILP input format (with dependencies, etc.)
    Problem = problem_generator.MOSAICProblem_ICAPS2019(
        ScenarioParams,
        base_station_name="base_station",
        TimeStep=1,
        minBandwidth=0.)
    Problem.buildScheduler()
    JSONinput = Problem.createJSON()

    # Get the task colors
    TaskColors = Problem.getTaskColors()

    return JSONinput, TaskColors


def evaluate_output(JSONinput, Schedule, verbose=False, take_sample_name='take_sample', analyze_sample_name='analyze_sample', store_sample_name='store_sample'):
    output_tasks = json.loads(Schedule)['tasks']
    input_tasks = json.loads(JSONinput)['Tasks']['OptionalTasks'].keys()

    query_names = [take_sample_name, analyze_sample_name, store_sample_name]

    outcome = {
        take_sample_name: {
            'available': 0,
            'done': 0,
        },
        analyze_sample_name: {
            'available': 0,
            'done': 0,
        },
        store_sample_name: {
            'available': 0,
            'done': 0,
        },
    }
    for task in input_tasks:
        for query in query_names:
            if task.find(query) != -1:
                outcome[query]['available'] += 1

    for task_dict in output_tasks:
        task = task_dict['name']
        for query in query_names:
            if task.find(query) != -1:
                outcome[query]['done'] += 1

    if verbose:
        print("Samples:\n - Taken: {}/{}\n - Analyzed: {}/{}\n - Stored: {}/{}".format(
            outcome['take_sample']['done'], outcome['take_sample']['available'],
            outcome['analyze_sample']['done'], outcome['analyze_sample']['available'],
            outcome['store_sample']['done'], outcome['store_sample']['available']))

    return outcome


if __name__ == "__main__":
    demo_input = demo_input(demo_type="assembly_line", network_size='large')
    json_description, task_colors = create_json_description(demo_input)
    print(json_description)
    print(task_colors)
