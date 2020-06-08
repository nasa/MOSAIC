"""
 Tools to transform contact plans to bandwidths

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

import numpy as np


def compute_time_varying_bandwidth(AgentNames, TVBandwidth, time_horizon, time_step):

    CommWindows = {}
    CommEnergyCost = {}
    Thor = int(float(time_horizon) / float(time_step))

    # THIS SHOULD BE RANGE(THOR). But below we liberally use the index of t as an actual time, not an int.
    # Maybe keep it as is now and fix at the end by changing the keys.

    # Initialize the object that will contain the bandwidth
    for t in range(Thor):  # np.arange(0,time_horizon+1,time_step):
        CommWindows[t] = {}
        CommEnergyCost[t] = {}
        for agent1 in AgentNames:
            CommWindows[t][agent1] = {}
            CommEnergyCost[t][agent1] = {}
            for agent2 in AgentNames:
                CommWindows[t][agent1][agent2] = 0.
                CommEnergyCost[t][agent1][agent2] = 0.

    # Find out what are the info_times of the links. We will use this as a proxy for "what time is it now".
    info_times = []
    for link in TVBandwidth:
        info_times.append(link["info_time"])
    if len(info_times):
        info_times = np.array(info_times)
        info_time = np.median(info_times)
        if np.max(np.abs(info_time-info_times)) > 1.:
            print("[Time-varying bandwidth computer] WARNING: links have very different info_times  (median: {}, max: {}, min: {})".format(
                info_time, np.max(info_times), np.min(info_times)))
    else:
        # No links, so no bandwidths Doesn't matter.
        info_time = 0.

    # Here, we use one extra time step at the end, since we are interested in what happens in each bucket.
    # We will only "charge" bandwidth to the first len(times)-1 steps.
    # Should be arange(info_time, info_time+time_horizon, time_step)
    times = np.arange(info_time, info_time+time_horizon+time_step, time_step)

    assert len(times) == Thor+1

    for link in TVBandwidth:
        # If the link is active during the time window of interest
        if link['bandwidth'] > 0 and link['time_start'] < np.max(times) and link['time_end'] > np.min(times):
            # Link start and end time. Window to times of interest
            link_time_start = np.max([link['time_start'], np.min(times)])
            link_end_time = np.min([link['time_end'], np.max(times)])
            # Extract link properties
            link_origin = link['origin']
            link_destination = link['destination']
            link_bandwidth = link['bandwidth']
            link_energy_cost = link['energy_cost']

            # Find the time step immediately after the start of the contact
            # Find the closest time
            just_after_time_start = np.argmin(
                np.abs(times-link_time_start))
            # If the closest discretized time is before the start time
            # pick the next time step
            if times[just_after_time_start] <= link_time_start:
                just_after_time_start += 1

            assert just_after_time_start <= len(
                times)-1, "ERROR: this link starts after the last step of times, and should not have been considered here"

            # How much of this link is inside the block immediately before just_after_time_start?
            assert just_after_time_start > 0, "ERROR: link_time_start is > min(times), so just_after_time_start should never be the first entry"

            duration_in_interval = times[just_after_time_start] - \
                link_time_start
            assert duration_in_interval >= 0, "ERROR: something wrong with calculation of durations in interval"
            # fraction_in_interval = float(first_block_length)/float(time_interval_duration)
            # ADD TO BW

            CommWindows[just_after_time_start -
                        1][link_origin][link_destination] += link_bandwidth*duration_in_interval
            CommEnergyCost[just_after_time_start -
                           1][link_origin][link_destination] += link_energy_cost

            for t_ix in range(just_after_time_start, len(times)-1):
                t = times[t_ix]
                if t > link_end_time:
                    break
                # We cheat at the end. If times are equally-spaced, all this is moot.
                if t_ix >= len(times)-1:
                    time_interval_duration = times[t_ix]-times[t_ix-1]
                else:
                    time_interval_duration = times[t_ix+1]-times[t_ix]
                duration_in_interval = np.min(
                    [link_end_time, t+time_interval_duration])-t
                # fraction_in_interval = float(duration_in_interval)/float(time_interval_duration)
                # ADD TO BW
                CommWindows[t_ix][link_origin][link_destination] += link_bandwidth * \
                    duration_in_interval
                CommEnergyCost[t_ix][link_origin][link_destination] += link_energy_cost

    return CommWindows, CommEnergyCost


def compute_time_invariant_bandwidth_legacy(AgentNames, TVBandwidth, time_horizon, time_step):
    Thor = int(float(time_horizon) / float(time_step))
    # Communication bandwidth
    od_pairs = []
    for link in TVBandwidth:
        od_pairs += [(link["origin"], link["destination"])]
    assert len(set(od_pairs)) == len(
        od_pairs), "ERROR: some links are repeated (is the problem time-varying?)"

    CommWindowsBandwidth = {}
    CommEnergyCost = {}
    for t in range(Thor):
        CommWindowsBandwidth[t] = {}
        CommEnergyCost[t] = {}
        # Initialize
        for agent1 in AgentNames:
            CommWindowsBandwidth[t][agent1] = {}
            CommEnergyCost[t][agent1] = {}
            for agent2 in AgentNames:
                CommWindowsBandwidth[t][agent1][agent2] = 0.
                CommEnergyCost[t][agent1][agent2] = 0.
        # Fill in
        for link in TVBandwidth:
            CommWindowsBandwidth[t][link["origin"]][link["destination"]
                                                    ] = link["bandwidth"]*float(time_step)
            CommEnergyCost[t][link["origin"]
                              ][link["destination"]] = link["energy_cost"]

    return CommWindowsBandwidth, CommEnergyCost


def compute_time_invariant_bandwidth(agents_list: list, TVBandwidth: dict, Thor: float):

    Bandwidth = {}
    Latency = {}
    EnergyCost = {}
    # agents_list = JSONInput["AgentCapabilities"]["MaxComputationLoad"].keys(
    # )

    for agent1 in agents_list:
        Bandwidth[agent1] = {}
        Latency[agent1] = {}
        EnergyCost[agent1] = {}
        for agent2 in agents_list:
            Bandwidth[agent1][agent2] = 0.
            Latency[agent1][agent2] = 0.
            EnergyCost[agent1][agent2] = 0.

    for link in TVBandwidth:
        if link["bandwidth"] == 0:
            continue
        _info_time = link.get('info_time', 0.)
        if link["origin"] not in Bandwidth.keys():
            Bandwidth[link["origin"]] = {}
            Latency[link["origin"]] = {}
            EnergyCost[link["origin"]] = {}
        if link["destination"] not in Bandwidth[link["origin"]].keys():
            Bandwidth[link["origin"]][link["destination"]] = 0.
            Latency[link["origin"]][link["destination"]] = 0.
            EnergyCost[link["origin"]][link["destination"]] = 0.

        # Compute fraction of the time horizon when the link is on
        link_windowed_time_start = max(link['time_start'], _info_time)
        link_windowed_end_time = min(link['time_end'], _info_time+Thor)
        link_effective_fraction = max(0., float(
            link_windowed_end_time-link_windowed_time_start)/float(Thor))
        # if link_effective_fraction == 0.:
        #     print(":( Link {} excluded at time {} (effective fraction {}, end_time {}, start_time {})".format(
        #         link, _info_time, link_effective_fraction, link_windowed_end_time, link_windowed_time_start))
        # else:
        #     print(":) Link {} active at time {} (effective fraction {})".format(
        #         link, _info_time, link_effective_fraction))

        # Add bandwidth
        Bandwidth[link["origin"]][link["destination"]
                                  ] += link["bandwidth"]*link_effective_fraction
        # Use average energy cost - will normalize at the end
        EnergyCost[link["origin"]][link["destination"]
                                   ] += link["energy_cost"]*link["bandwidth"]*link_effective_fraction
        # Use average latency - will normalize at the end
        Latency[link["origin"]][link["destination"]] += (link["latency"]+max(
            0., link['time_start']-_info_time))*link["bandwidth"]*link_effective_fraction
    # # Now normalize
    for agent1 in EnergyCost.keys():
        for agent2 in EnergyCost[agent1].keys():
            if float(Bandwidth[agent1][agent2]) > 0:
                EnergyCost[agent1][agent2] /= float(Bandwidth[agent1][agent2])
                Latency[agent1][agent2] /= float(Bandwidth[agent1][agent2])

    # allzeros = True
    # for a1 in Bandwidth.keys():
    #     for a2 in Bandwidth[a1].keys():
    #         if Bandwidth[a1][a2] > 0:
    #             allzeros = False
    #             print("BW[{}][{}]={}".format(a1, a2, Bandwidth[a1][a2]))
    # if allzeros:
    #     print("All bandwidths are zero!")

    return Bandwidth, Latency, EnergyCost

    # CommNet = CommunicationNetwork(
    #     Bandwidth=Bandwidth,
    #     Latency=Latency,
    #     EnergyCost=EnergyCost,
    # )
