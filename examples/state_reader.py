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


import random
import numpy as np
import json
import argparse
import matplotlib.pyplot as plt
import copy


def create_scenario(Agent, agent_states, network_state, current_time=1001, time_horizon=60):
    if type(agent_states) is str:
        agent_states = json.loads(agent_states)  # Ensure we have a dict
    else:
        # No changes to the original input
        agent_states = copy.deepcopy(agent_states)

    if type(network_state) is str:
        network_state = json.loads(network_state)  # Ensure we have a dict
    else:
        # No changes to the original input
        network_state = copy.deepcopy(network_state)

    Agents = agent_states.keys()
    CommunicationNetwork = network_state

    for agent in agent_states.keys():
        if agent_states[agent].get('in_science_zone', False) is True:
            agent_states[agent]['samples'] = 1
        else:
            agent_states[agent]['samples'] = 0

    # Agents' capabilities
    ComputationTime = {}
    EnergyCost = {}
    battery_computation_time_multiplier = {
        0: None,
        1: None,
        2: 1.5,
        3: 1.,
        4: 0.75,
    }
    battery_energy_cost_multiplier = {
        0: 100.,
        1: 100.,
        2: 5.,
        3: 2.,
        4: 1.,
    }

    battery_bandwidth = {
        0: 0.,
        1: 0.,
        2: 0.5,
        3: 10.,
        4: None,
    }

    # Remove dead agents and related link
    good_battery_agents = []
    for agent in Agents:
        battery_level = agent_states[agent]['battery_discrete_level']
        if battery_level > 1:
            good_battery_agents.append(agent)

    Agents = good_battery_agents

    NewCommNetwork = []
    for link in CommunicationNetwork:
        battery_level_tx = agent_states[link['origin']
                                        ]['battery_discrete_level']
        battery_level_rx = agent_states[link['destination']
                                        ]['battery_discrete_level']
        if battery_level_tx > 1 and battery_level_rx > 1:
            NewCommNetwork.append(link)
    CommunicationNetwork = NewCommNetwork

    for agent in Agents:
        battery_level = agent_states[agent]['battery_discrete_level']
        if battery_level <= 1:  # Can't do anything
            ComputationTime[agent] = {}
            EnergyCost[agent] = {}
        else:
            base_station = False
            if agent.find('base_station') != -1:
                base_station = True
            if ('is_base_station' in agent_states[agent].keys()):
                if (agent_states[agent]['is_base_station'] is True):
                    base_station = True
                else:
                    base_station = False
            # base_station = False

            # If the agent name contains "base_station"
            if base_station is True:
                ComputationTime[agent] = {
                    "vo_localization": 3. * battery_computation_time_multiplier[battery_level],
                    "analyze_sample": 3. * battery_computation_time_multiplier[battery_level],
                    "store_sample": .1 * battery_computation_time_multiplier[battery_level]
                }
                EnergyCost[agent] = {
                    "vo_localization": 0.03333333333333333 * battery_energy_cost_multiplier[battery_level],
                    "analyze_sample": 0.03333333333333333 * battery_energy_cost_multiplier[battery_level],
                    "store_sample": 0.003333333333333333 * battery_energy_cost_multiplier[battery_level]
                }
            else:
                ComputationTime[agent] = {
                    "short_range_image": 3 * battery_computation_time_multiplier[battery_level],
                    "vo_localization": 12 * battery_computation_time_multiplier[battery_level],
                    "post_vo_localization": 0.001 * battery_computation_time_multiplier[battery_level],
                    "take_sample": 5 * battery_computation_time_multiplier[battery_level],
                    "analyze_sample": 13 * battery_computation_time_multiplier[battery_level],
                }
                EnergyCost[agent] = {
                    "short_range_image": 0.13333333 * battery_energy_cost_multiplier[battery_level],
                    "vo_localization": 0.3333333333333333 * battery_energy_cost_multiplier[battery_level],
                    "post_vo_localization": 0.00000001 * battery_energy_cost_multiplier[battery_level],
                    "take_sample": 0.2 * battery_energy_cost_multiplier[battery_level],
                    "analyze_sample": 0.3333333333333333 * battery_energy_cost_multiplier[battery_level],
                }

    # Adapt communication network to reduced bandwidth
    for link in CommunicationNetwork:
        battery_level_tx = agent_states[link['origin']
                                        ]['battery_discrete_level']
        battery_level_rx = agent_states[link['destination']
                                        ]['battery_discrete_level']
        min_battery_level = min(battery_level_tx, battery_level_rx)
        if min_battery_level < 4:
            link['bandwidth'] = min(
                link['bandwidth'], battery_bandwidth[min_battery_level])

    # Hard-coded things
    ProductsSize = {
        "short_range_image": 8.0,
        "vo_localization": 0.1,
        "post_vo_localization": 0.0001,
        "take_sample": 15,
        "analyze_sample": 1.,
        "store_sample": 0.1
    }
    TaskReward = {
        "short_range_image": 0,
        "vo_localization": 0,
        "post_vo_localization": 0,
        "take_sample": 1,
        "analyze_sample": 10,
        "store_sample": 20
    }

    CostFunction = {
        "energy": 0.5,
        "total_task_reward": 1.0,
        "total_time": 0.
    }

    # Put it all together
    ScenarioParameters = {
        "Agent": Agent,
        "Agents": Agents,
        "AgentStates": agent_states,
        "AgentCapabilities": {
            "ComputationTime": ComputationTime,
            "EnergyCost": EnergyCost,
        },
        "CommunicationNetwork": CommunicationNetwork,
        "Options": {},
        "Tasks": {
            "ProductsSize": ProductsSize,
            "TaskReward": TaskReward,
            "DomainSpecific": {}
        },
        "Time": {
            "CurrentTime": current_time,
            "TimeHorizon": time_horizon
        },
        "CostFunction": CostFunction
    }

    return json.dumps(ScenarioParameters)


if __name__ == "__main__":
    problem_state = {
        'time': 1002.0,
        'Agent': 'puffer1',
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
            'puffer3': {'in_science_zone': False, 'battery_discrete_level': 4, 'battery_level': 77},
            'puffer2': {'in_science_zone': False, 'battery_discrete_level': 3, 'battery_level': 51},
            'puffer1': {'in_science_zone': True,  'battery_discrete_level': 2, 'battery_level': 27},
            'base_station': {'in_science_zone': False,  'battery_discrete_level': 3, 'battery_level': 51, 'is_base_station': True},
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
    communication_network = []
    for link in problem_state['link_state'].keys():
        props = problem_state['link_state'][link]
        communication_network.append(
            {"bandwidth": props['bandwidth'],
                "destination": link[1],
                "energy_cost": 0.0,
                "latency": 0.001,
                "origin": link[0],
                "time_end": 1.7976931348623157e+308,
                "time_start": 0.}
        )

    agent_states = problem_state["nodes"]

    scenario_params = create_scenario(
        Agent=problem_state["Agent"],
        agent_states=json.dumps(agent_states),
        network_state=json.dumps(communication_network),
        current_time=problem_state["time"],
        time_horizon=problem_state["time_horizon"]
    )
    print(scenario_params)
