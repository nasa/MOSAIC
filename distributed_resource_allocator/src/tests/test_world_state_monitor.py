#!/usr/bin/python
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

import json
import defaults_scaffolding as def_vals
import rospy
from std_msgs.msg import String, Int32

class world_state_monitor:
    def __init__(self, rate=def_vals.PUB_RATE):

        self.agent_state = {
            'node_91': {'energy_usage_tx': 4, 'energy_usage_rx': 2, 'battery_capacity': 2000, 'battery_level': 2000, 'battery_discrete_level': 4,
                       'data': [], 'backlog_volume': 0},
            'node_92': {'energy_usage_tx': 4, 'energy_usage_rx': 2, 'battery_capacity': 2000, 'battery_level': 2000, 'battery_discrete_level': 4,
                       'data': [], 'backlog_volume': 0},#{'id': 'd1', 'size': 2, 'destination': 'node_4'}]},
            # 'node_93': {'energy_usage_tx': 4, 'energy_usage_rx': 2, 'battery_capacity': 200, 'battery_level': 100,
            #            'data': []},
            'base_station': {'energy_usage_tx': 4, 'energy_usage_rx': 2, 'battery_capacity': 20000, 'battery_level': 20000, 'battery_discrete_level': 4,
                       'data': [], 'backlog_volume': 0}}

        self.network_state = [
                {"bandwidth": 11.0,
                 "destination": "base_station",
                 "energy_cost": 0.0,
                 "latency": 0.001,
                 "origin": "node_91",
                 "time_end": 1.7976931348623157e+308,
                 "time_start": 0.0},
                {"bandwidth": 11.0,
                 "destination": "node_91",
                 "energy_cost": 0.0,
                 "latency": 0.001,
                 "origin": "base_station",
                 "time_end": 1.7976931348623157e+308,
                 "time_start": 0.0},
                {"bandwidth": 11.0,
                 "destination": "base_station",
                 "energy_cost": 0.0,
                 "latency": 0.001,
                 "origin": "node_92",
                 "time_end": 1.7976931348623157e+308,
                 "time_start": 0.0},
                {"bandwidth": 11.0,
                 "destination": "node_92",
                 "energy_cost": 0.0,
                 "latency": 0.001,
                 "origin": "base_station",
                 "time_end": 1.7976931348623157e+308,
                 "time_start": 0.0},
                {"bandwidth": 11.0,
                 "destination": "node_91",
                 "energy_cost": 0.0,
                 "latency": 0.001,
                 "origin": "node_91",
                 "time_end": 1.7976931348623157e+308,
                 "time_start": 0.0},
                {"bandwidth": 11.0,
                 "destination": "node_92",
                 "energy_cost": 0.0,
                 "latency": 0.001,
                 "origin": "node_92",
                 "time_end": 1.7976931348623157e+308,
                 "time_start": 0.0},
                {"bandwidth": 11.0,
                 "destination": "node_91",
                 "energy_cost": 0.0,
                 "latency": 0.001,
                 "origin": "node_92",
                 "time_end": 1.7976931348623157e+308,
                 "time_start": 0.0},
                {"bandwidth": 11.0,
                 "destination": "node_92",
                 "energy_cost": 0.0,
                 "latency": 0.001,
                 "origin": "node_91",
                 "time_end": 1.7976931348623157e+308,
                 "time_start": 0.0},
            ]

        # Rate at which world plan is published
        self.rate_ctrl = rospy.Rate(rate)
        # Publisher for agent state
        self.pub_agent = rospy.Publisher('/agent_state', String,
                                         queue_size=def_vals.PUB_QUEUE_SIZE)

        self.pub_network = rospy.Publisher('/contact_plan', String,
                                           queue_size=def_vals.PUB_QUEUE_SIZE)

        # HACK to switch allocation
        rospy.Subscriber('/all/distware/wsm/switch', Int32, self._switch_wsm)
        self.network_state_switch = 1

        # Fetch topics from other agents to find out who is active
        self.topics = rospy.get_published_topics()

    def _switch_wsm(self, msg):
        def _create_link(origin, destinaton):
            link = {"bandwidth": 11.0,
                    "destination": destinaton,
                    "energy_cost": 0.0,
                    "latency": 0.001,
                    "origin": origin,
                    "time_end": 1.7976931348623157e+308,
                    "time_start": 0.}
            return link
        _comm_network = [_create_link(
            'node_91', 'node_91'), _create_link('node_92', 'node_92')]
        if msg.data == 0:
            # Everyone can talk to everyone
            _comm_network.append(_create_link('node_91', 'node_92'))
            _comm_network.append(_create_link('node_92', 'node_91'))
            _comm_network.append(_create_link('base_station', 'node_91'))
            _comm_network.append(_create_link('node_91', 'base_station'))
            _comm_network.append(_create_link('base_station', 'node_92'))
            _comm_network.append(_create_link('node_92', 'base_station'))
        elif msg.data == 1:
            # 1 can talk to base
            _comm_network.append(_create_link('base_station', 'node_91'))
            _comm_network.append(_create_link('node_91', 'base_station'))
        elif msg.data == 2:
            # 2 can talk to base
            _comm_network.append(_create_link('base_station', 'node_92'))
            _comm_network.append(_create_link('node_92', 'base_station'))
        elif msg.data == 3:
            # 1 can talk to bs through 2
            _comm_network.append(_create_link('node_91', 'node_92'))
            _comm_network.append(_create_link('node_92', 'node_91'))
            _comm_network.append(_create_link('base_station', 'node_92'))
            _comm_network.append(_create_link('node_92', 'base_station'))
        else:
            # Disconnected
            pass
        self.network_state = {"CommunicationNetwork": _comm_network}

    def run(self):
        # Do until ROS finishes
        while not rospy.is_shutdown():
            # Publish a string mindlessly
            self.pub_agent.publish(json.dumps(
                self.agent_state, sort_keys=True))
            self.pub_network.publish(json.dumps(
                self.network_state, sort_keys=True))
            # Wait for a while
            self.rate_ctrl.sleep()


if __name__ == "__main__":
    # Start ROS node
    rospy.init_node('wsm_test', anonymous=True)

    # Get the rate
    rate = rospy.get_param('~rate', def_vals.PUB_RATE)

    # Create and run WSM
    wsm = world_state_monitor(rate)
    wsm.run()
