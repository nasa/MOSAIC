"""
Scenario generator for the PUFFER scheduling problem.
Creates a configuration file that will be digested by solver-specific parsers.

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


def create_scenario(
    num_puffers=None,
    base_station=None,
    science_zone_probability=0.3,
    science_samples_per_puffer=None,
    plot_scenario=False,
    plot_fig_to_screen=True,
    save_fig_to_file=False,
    save_fig_path='Scenario.png',
):
    '''
    Creates a scenario input file for the PUFFER MOSAIC problem.
    Inputs:
    - num_puffers, the number of PUFFERs present. If no number is specified,
     a random number between 1 and 15 is selected.
    - base_station, whether a base station is present. If no bool is specified,
    the presence of the base station is randomly decided with a 50/50 chance.
    - science_zone_probability [default=0.3], the likelihood that a PUFFER is
    in a science zone. Conditional on being in the science zone, each PUFFER has
    a uniform probability of drawing one through five samples.
    '''

    # Decide on a number of PUFFERs and whether there is a base station or not
    if num_puffers is None:
        num_puffers = random.randint(1, 15)
    else:
        num_puffers = int(num_puffers)
    puffer_names = []
    for puffer in range(num_puffers):
        puffer_names.append('puffer{}'.format(puffer+1))
    if base_station is None:
        base_station = random.choice([True, False])

    science_samples = {}
    AgentStates = {}

    Agents = puffer_names
    if base_station:
        Agents = ['base_station'] + Agents
        AgentStates['base_station'] = {}

    agent_locations = {}
    region_size = 10.

    # Decide on the number of science samples available
    for agent in Agents:
        agent_locations[agent] = [random.random()*region_size,
                                  random.random()*region_size]
        if agent != 'base_station':
            puffer = agent
            in_science_zone = (random.random() < science_zone_probability)
            if in_science_zone:
                if science_samples_per_puffer is None:
                    _science_samples = random.randint(1, 5)
                else:
                    _science_samples = science_samples_per_puffer
                science_samples[puffer] = _science_samples
            else:
                science_samples[puffer] = 0
            AgentStates[puffer] = {
                "in_science_zone": in_science_zone, "samples": science_samples[puffer]}

    # Decide which robot is making the call
    # Agent = random.choice(Agents)
    Agent = Agents[0]

    # Draw communication links

    # Bandwidth and communication network
    CommunicationNetwork = []
    for agent1 in Agents:
        for agent2 in Agents:
            if agent1 == agent2:
                bw = 11.
            else:
                bw = compute_bandwidth(
                    agent_locations[agent1], agent_locations[agent2], mode='table')
            CommunicationNetwork.append(
                {"bandwidth": bw,
                 "destination": agent2,
                 "energy_cost": 0.0,
                 "latency": 0.001,
                 "origin": agent1,
                 "time_end": 1.7976931348623157e+308,
                 "time_start": 0.}
            )

    # Agents' capabilities
    ComputationTime = {}
    EnergyCost = {}
    for agent in Agents:
        if agent == 'base_station':
            ComputationTime[agent] = {
                "vo_localization": 1.,
                "plan_path": .1,
                "analyze_sample": 1.,
                "store_sample": .1
            }
            EnergyCost[agent] = {
                "vo_localization": 0.03333333333333333,
                "plan_path": 0.03333333333333333,
                "analyze_sample": 0.03333333333333333,
                "store_sample": 0.003333333333333333
            }
        else:
            ComputationTime[agent] = {
                "short_range_image": 3,
                "vo_localization": 10,
                "plan_path": 10,
                "send_drive_cmd": 0.1,
                "take_sample": 5,
                "analyze_sample": 10
            }
            EnergyCost[agent] = {
                "short_range_image": 0.13333333,
                "vo_localization": 0.3333333333333333,
                "plan_path": 0.3333333333333333,
                "send_drive_cmd": 0.06666666666666667,
                "take_sample": 0.2,
                "analyze_sample": 0.3333333333333333
            }

    # Hard-coded things
    ProductsSize = {
        "short_range_image": 8.0,
        "vo_localization": 0.1,
        "plan_path": 0.1,
        "send_drive_cmd": 0.1,
        "take_sample": 15,
        "analyze_sample": 1.,
        "store_sample": 0.1
    }
    TaskReward = {
        "short_range_image": 0,
        "vo_localization": 0,
        "plan_path": 0,
        "send_drive_cmd": 0,
        "take_sample": 0,
        "analyze_sample": 10,
        "store_sample": 20
    }

    CostFunction = {
        "energy": 0,
        "total_task_reward": 1.0,
        "total_time": 0.5
    }

    # Put it all together
    ScenarioParameters = {
        "Agent": Agent,
        "Agents": Agents,
        "AgentStates": AgentStates,
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
            "CurrentTime": 1001,
            "TimeHorizon": 60
        },
        "CostFunction": CostFunction
    }

    if plot_scenario:
        scenario_plotter(
            AgentStates,
            agent_locations,
            CommunicationNetwork,
            region_size,
            plot_to_screen=plot_fig_to_screen,
            save_to_file=save_fig_to_file,
            save_path=save_fig_path,
        )

    return json.dumps(ScenarioParameters)


def scenario_plotter(AgentStates,
                     agent_locations,
                     CommunicationNetwork,
                     region_size,
                     plot_to_screen=True,
                     save_to_file=False,
                     save_path='scenario.png',
                     ):
    import matplotlib.pyplot as plt
    _, ax = plt.subplots()
    for link in CommunicationNetwork:
        ax.plot(
            [agent_locations[link['origin']][0],
                agent_locations[link['destination']][0]],
            [agent_locations[link['origin']][1],
                agent_locations[link['destination']][1]],
            linewidth=link['bandwidth']*.3,
            color=[.7, .7, .7])
    for agent in agent_locations:
        if agent == 'base_station':
            markertype = '.y'
            marker_size = 30
        else:
            markertype = '.r'
            if AgentStates[agent]["in_science_zone"]:
                marker_size = 10*AgentStates[agent]["samples"]
            else:
                markertype = '.k'
                marker_size = 10
        ax.plot(agent_locations[agent][0], agent_locations[agent]
                [1], markertype, markersize=marker_size)

    ax.axis('equal')
    ax.set_xlim(left=-1, right=region_size+1)
    ax.set_ylim(bottom=-1, top=region_size+1)
    plt.axis('off')
    if plot_to_screen:
        plt.show()
    if save_to_file:
        plt.savefig(save_path)


def compute_bandwidth(l1, l2, unit_distance=1, bandwidth_boost=1., mode='l2'):
    l1 = np.array(l1) * unit_distance
    l2 = np.array(l2) * unit_distance
    dist = np.linalg.norm(l1 - l2)
    if mode == 'l2':
        base_bw = 0.25
        base_distance = 100.  # 250 kb at 100 m
        bw = base_bw / (dist / base_distance)**2
    elif mode == 'table':
        if dist < 5.:
            bw = 11.
        elif dist < 11.:
            bw = 5.5
        elif dist < 50.:
            bw = 1
        elif dist < 200.:
            bw = 0.1
        else:
            bw = 0.
    else:
        print("Bandwidth mode unsupported")
        bw = 0.
    return bw * bandwidth_boost


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Generates a scenario parameters file for the PUFFER FY18 demo MOSAIC scenario.')
    parser.add_argument('--num_puffers', default=None,
                        help="The number of PUFFERs in the scenario")
    parser.add_argument('--base_station_present', action='store_true',
                        help='Whether a base station is present')
    parser.add_argument('--science_zone_probability', default=0.3,
                        help="The likelihood that a PUFFER is in a science zone")
    parser.add_argument('--random_seed', default=None,
                        help="The random seed for the scenario")
    parser.add_argument('--Plot', action='store_true',
                        help='Plot the scenario and the schedule')

    args = parser.parse_args()

    if args.random_seed is not None:
        random.seed(args.random_seed)

    ScenarioParams = create_scenario(
        num_puffers=args.num_puffers,
        base_station=args.base_station_present,
        science_zone_probability=args.science_zone_probability,
        plot_scenario=args.Plot)
    print(ScenarioParams)
