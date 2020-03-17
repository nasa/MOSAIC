#!/usr/bin/python
"""
 Copyright 2020 by California Institute of Technology.  ALL RIGHTS RESERVED.
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

import rospy
from pdra.msg import CommData
from std_msgs.msg import String
from threading import Lock
import defaults_scaffolding as def_vals
import ast
from time import sleep


class p2p_dtn_simulator(object):
    """ A class to simulate one-hop delay-tolerant networking in a ROS-based environment """

    # def __init__(self, contact_plan: list, start_time_offset: float = 0, contact_plan_topic: str = '/network_topology', sleep_interval: float = 0.1):
    def __init__(self, contact_plan, start_time_offset=0, contact_plan_topic='/network_topology', sleep_interval=0.1):
        """ Inputs:
        - contact_plan, a list of contacts. Each entry is a dictionary containing:
          - "origin", the name of the transmitter
          - "destination", the name of the receiver
          - "time_start", the start time of the contact
          - "time_end", the end time of the contact
          - "bandwidth", the available bandwidth.
          A contact is feasible if bandwidth>0 and the current time is between start_time and end_time
        - start_time_offset. The simulator grabs the current time via `rospy.get_time()`.
          On the other hand, time_start and time_end are often specified with respect to the simulation start time.
          An offset equal to start_time_offset is applied to the clock to compensate for this.
        - contact_plan_topic, the name of a ROS topic where contact_plan updates will be published.
          For compatibility reasons, the updates are expected to be a JSON string containing a dictionary
          with the key "contact_plan" and value corresponding to the contact_plan described above.
        - sleep_interval, a float. The internal loop will sleep for this amount of time to avoid busylooping.
        """
        self.agents = {}
        # Lock on agent publishers. Makes sure we do not delete a publisher in the callback while using it in loop()

        self.pub_lock = Lock()
        self.start_time_offset = start_time_offset
        self.sleep_interval = sleep_interval
        self.enqueued_messages = {}
        self.agent_subscriptions = {}
        self.agent_publishers = {}
        self.messages_in_transmission = []

        self.update_contact_plan(contact_plan)

        # Subscribe to contact plan updates
        self.contact_plan_updater = rospy.Subscriber(
            contact_plan_topic, String, self.contact_plan_msg_update)

    def contact_plan_msg_update(self, msg):
        self.logdebug("Received new contact plan")
        new_network_topology = ast.literal_eval(msg.data)
        new_contact_plan = new_network_topology["contact_plan"]
        self.update_contact_plan(new_contact_plan)

    def update_contact_plan(self, new_contact_plan):
        self.contact_plan = new_contact_plan
        agents_list = set()
        for link in self.contact_plan:
            agents_list.add(link["origin"])
            agents_list.add(link["destination"])
        self.agents = {agent: {
            "rx": "/{}/comm/rx".format(agent),
            "tx": "/{}/comm/tx".format(agent),
        } for agent in agents_list
        }
        self._update_enqueued_messages()
        self._update_subscriptions()

    def _update_enqueued_messages(self):
        # Add new agents
        for tx_agent in self.agents.keys():
            if tx_agent not in self.enqueued_messages.keys():
                self.enqueued_messages[tx_agent] = {rx_agent: []
                                                    for rx_agent in self.agents.keys()}
            else:
                for rx_agent in self.agents.keys():
                    if rx_agent not in self.enqueued_messages[tx_agent].keys():
                        self.enqueued_messages[tx_agent][rx_agent] = []
        # Remove stale agents
        tx_agents_to_pop = []
        for tx_agent in self.enqueued_messages.keys():
            if tx_agent not in self.agents.keys():
                self.loginfo(
                    "Agent {} is no longer active, removing".format(tx_agent))
                for rx_agent in self.enqueued_messages[tx_agent].keys():
                    if len(self.enqueued_messages[tx_agent][rx_agent]):
                        self.logwarn("Dropping messages from removed agent {} to agent {}".format(
                            tx_agent, rx_agent))
                tx_agents_to_pop.append(tx_agent)
            else:
                rx_agents_to_pop = []
                for rx_agent in self.enqueued_messages[tx_agent].keys():
                    if rx_agent not in self.agents.keys():
                        self.loginfo(
                            "Agent {} is no longer active, removing".format(rx_agent))
                        rx_agents_to_pop.append(rx_agent)
                for topop_rx in rx_agents_to_pop:
                    self.enqueued_messages[tx_agent].pop(topop_rx)
        for topop_tx in tx_agents_to_pop:
            self.enqueued_messages.pop(topop_tx)

    def _update_subscriptions(self):
        for agent_name, agent_topics in self.agents.iteritems():
            if agent_name not in self.agent_subscriptions.keys():
                self.agent_subscriptions[agent_name] = rospy.Subscriber(
                    agent_topics['tx'], CommData, self.receive, agent_name)
            if agent_name not in self.agent_publishers.keys():
                self.pub_lock.acquire()
                self.agent_publishers[agent_name] = rospy.Publisher(
                    agent_topics['rx'], CommData, queue_size=def_vals.PUB_QUEUE_SIZE)
                self.pub_lock.release()
        for agent_name in self.agent_subscriptions.keys():
            if agent_name not in self.agents.keys():
                self.agent_subscriptions[agent_name].unregister()
                self.agent_subscriptions.pop(agent_name)
        for agent_name in self.agent_publishers.keys():
            if agent_name not in self.agents.keys():
                self.agent_publishers.pop(agent_name)

    # def receive(self, transmitter: str, msg: CommData):
    def receive(self, msg, transmitter):
        # Append message to msg queue. The loop function will dequeue if needed.
        receiver = msg.receiver
        self.logdebug("Received message from node {} to node {}".format(
            transmitter, receiver))
        self.enqueued_messages[transmitter][receiver].append(msg)
        self.logdebug("There are {} messages in the queue from node {} to node {}".format(len(self.enqueued_messages[transmitter][receiver]),
                                                                                          transmitter, receiver))

    def loop(self):
        # Get ROS time
        current_time = rospy.get_time() - self.start_time_offset
        # Loop through links
        for link in self.contact_plan:
            # If link is active now
            if link["time_start"] < current_time and link["time_end"] > current_time:
                # Publish all messages
                while self.enqueued_messages[link["origin"]][link["destination"]]:
                    assert link["destination"] in self.agent_publishers.keys(
                    ), "ERROR: receiver {} is not in agent publishers".format(link["destination"])
                    self.pub_lock.acquire()
                    rospy.logdebug("Forwarding message from {} to {}".format(
                        link["origin"], link["destination"]))
                    msg = self.enqueued_messages[link["origin"]
                                                 ][link["destination"]].pop()
                    # self.agent_publishers[link["destination"]].publish(msg)
                    msg_wrapper = {
                        'receive_time': current_time + len(msg.data)/(link["bandwidth"]*1e3),
                        'destination': link["destination"],
                        'msg': msg
                    }
                    self.messages_in_transmission.append(msg_wrapper)
                    self.logdebug("Message transmission time is {} s (size {} bits, bandwidth {} Mbps)\n Will be received at {} (now is {})".format(
                        len(msg.data)/(link["bandwidth"]*1e3), len(msg.data)*1e3, link["bandwidth"], msg_wrapper['receive_time'], current_time))
                    self.pub_lock.release()
        ready_msgs = [
            msg_wrapper for msg_wrapper in self.messages_in_transmission if msg_wrapper['receive_time'] <= current_time]
        for msg_wrapper in ready_msgs:
            self.logdebug("Sending message")
            self.agent_publishers[msg_wrapper["destination"]].publish(
                msg_wrapper['msg'])
            # Assumption: msg_wrappers are unique
            self.messages_in_transmission.remove(msg_wrapper)

        for origin, destinations in self.enqueued_messages.iteritems():
            for destination, msgs in destinations.iteritems():
                if len(msgs):
                    rospy.loginfo("{} messages enqueued from {} to {}".format(
                        len(msgs), origin, destination))

    def run(self):
        while not rospy.is_shutdown():
            self.loop()
            sleep(self.sleep_interval)

    def logerr(self, msg):
        rospy.logerr("[P2P DTN simulator]: {}".format(msg))

    def logwarn(self, msg):
        rospy.logwarn("[P2P DTN simulator]: {}".format(msg))

    def loginfo(self, msg):
        rospy.loginfo("[P2P DTN simulator]: {}".format(msg))

    def logdebug(self, msg):
        rospy.logdebug("[P2P DTN simulator]: {}".format(msg))


if __name__ == "__main__":
    rospy.init_node('p2p_dtn_simulator', anonymous=True)

    contact_plan = ast.literal_eval(rospy.get_param('~contact_plan', '[]'))
    contact_plan_topic = rospy.get_param(
        '~network_topology_topic', '/network_topology')
    agents = ast.literal_eval(rospy.get_param(
        '~agents', '[]'))

    simple_dtn_sim = p2p_dtn_simulator(
        contact_plan=contact_plan,
        start_time_offset=rospy.get_time(),
        contact_plan_topic=contact_plan_topic
    )
    simple_dtn_sim.run()
