#!/usr/bin/python

"""
# =====================================================================
# RAC dispatcher class
#
# Author: Federico Rossi
# Date:   02/13/2019

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

=====================================================================

"""

import json
import rospy
from std_msgs.msg import String, Int32


class abstract_resource_allocation_matcher():
    def __init__(
        self,
        agent_id,
        agent_state_topic='/all/distware/agent_state',
        network_state_topic='/all/distware/network_state',
        problem_description_topic='/all/distware/problem_description'
    ):
        # Things to keep track of
        self.agent_id = agent_id
        self.agent_state = None
        self.network_state = None
        self.problem_description = None
        self.alloc = {}
        # Will publish resource-allocation match here
        # Only keep the most recent solution
        self.pub_rac = rospy.Publisher('output', String, queue_size=1)
        # Listen to agents' state containing AgentStates and AgentCapabilities
        rospy.Subscriber(agent_state_topic,
                         String, self._process_agent_state)

        # Listen to network state containing CommunicationNetwork
        rospy.Subscriber(network_state_topic,
                         String, self._process_network_state)
        # Listen to problem description containing Options, Tasks, Time, CostFunction
        rospy.Subscriber(problem_description_topic,
                         String, self._process_problem_description)

    def _process_agent_state(self, data):
        _agent_state = json.loads(data.data)
        if self.agent_state != _agent_state:
            self.loginfo("New agent states")
            self.agent_state = _agent_state
            self.compute_rac()

    def _process_network_state(self, data):
        _network_state = json.loads(data.data)
        if self.network_state != _network_state:
            self.loginfo("New network state")
            self.network_state = json.loads(data.data)
            self.compute_rac()

    def _process_problem_description(self, data):
        _problem_description = json.loads(data.data)
        if json.dumps(self.problem_description) != json.dumps(_problem_description):
            self.loginfo("New problem description")
            self.problem_description = json.loads(data.data)
            self.compute_rac()

    def compute_rac(self):
        raise NotImplementedError

    def logdebug(self, msg):
        rospy.logdebug("[{}/RAC]: {}".format(self.agent_id, msg))

    def loginfo(self, msg):
        rospy.loginfo("[{}/RAC]: {}".format(self.agent_id, msg))

    def logwarn(self, msg):
        rospy.logwarn("[{}/RAC]: {}".format(self.agent_id, msg))

    def logerr(self, msg):
        rospy.logerr("[{}/RAC]: {}".format(self.agent_id, msg))

    def run(self, rate=.2):
        # rospy.spin()
        rate_ctrl = rospy.Rate(rate)
        while not rospy.is_shutdown():
            if self.alloc is not None and self.alloc != {}:
                self.pub_rac.publish(json.dumps(self.alloc))
            rate_ctrl.sleep()


class example_resource_allocation_matcher(abstract_resource_allocation_matcher):
    def __init__(
        self,
        agent_id,
        agent_state_topic='/all/distware/agent_state',
        network_state_topic='/all/distware/network_state',
        problem_description_topic='/all/distware/problem_description'
    ):
        abstract_resource_allocation_matcher.__init__(
            self,
            agent_id,
            agent_state_topic=agent_state_topic,
            network_state_topic=network_state_topic,
            problem_description_topic=problem_description_topic
        )
        # HACK to switch allocation
        rospy.Subscriber('/all/distware/rac/switch', Int32, self._switch_rac)
        self.alloc = {'nav':   'agent1',
                      'store': 'agent2'}

    def compute_rac(self):
        self.loginfo("Computed new resource allocation")
        self.pub_rac.publish(json.dumps(self.alloc))

    def _switch_rac(self, msg):
        if msg.data > 0:
            self.alloc = {'nav':   'agent1',
                          'store': 'agent2'}
        else:
            self.alloc = {'nav':   'agent2',
                          'store': 'agent2'}
        self.compute_rac()


class static_resource_allocation_matcher(abstract_resource_allocation_matcher):
    def __init__(
        self,
        agent_id,
        alloc,
        agent_state_topic='/all/distware/agent_state',
        network_state_topic='/all/distware/network_state',
        problem_description_topic='/all/distware/problem_description'
    ):
        abstract_resource_allocation_matcher.__init__(
            self,
            agent_id,
            agent_state_topic=agent_state_topic,
            network_state_topic=network_state_topic,
            problem_description_topic=problem_description_topic
        )        # HACK to switch allocation
        rospy.Subscriber('/all/distware/rac/switch', Int32, self._switch_rac)
        self.default_alloc = alloc
        self.alloc = alloc

    def compute_rac(self):
        self.loginfo("Computed new resource allocation")
        self.pub_rac.publish(json.dumps(self.alloc))

    def _switch_rac(self, msg):
        if msg.data > 0:
            self.alloc = self.default_alloc
        else:
            alloc = self.default_alloc.copy()
            for task in self.default_alloc.items():
                self.default_alloc[task] = self.agent_id
            self.alloc = alloc
        self.compute_rac()


class img_nav_drive_resource_allocation_matcher(abstract_resource_allocation_matcher):
    def compute_rac(self):
        from mosaic_schedulers.schedulers import ti_milp
        from mosaic_schedulers.common.examples.nav_plan_drive import MILPProblemGenerator as ProblemGenerator
        # TODO logic to ensure we do not spawn too many rac calculations
        # Create a problem in MOSAIC format
        if self.agent_state is not None and self.problem_description is not None and self.network_state is not None:
            self.loginfo("Computing new resource allocation.")
            ScenarioDescription = self.agent_state.copy()
            ScenarioDescription.update(self.problem_description.copy())
            ScenarioDescription.update(self.network_state.copy())
            ScenarioDescription.update({"Agent": self.agent_id})
            ScenarioDescription.update({"Agents": self.agent_state.keys()})
            # ScenarioDescription["Time"]["CurrentTime"] = rospy.get_time()

            Problem = ProblemGenerator.MOSAICProblem(
                json.dumps(ScenarioDescription),
                minBandwidth=0.,
                solverClockLimit=10,
            ).createJSON()
            # Compute the allocation
            Schedule = ti_milp.JSONSolver(Problem, solver='SCIP').schedule()
            # Here we use the names from the scheduler - which may not match the ones from the Autonomy.
            # That is OK - we handle it in the Dispatcher.
            # Output the allocation to a map
            Schedule = json.loads(Schedule)['tasks']
            _alloc = {}
            for task in Schedule:
                if task["id"] != "transfer_data" and task["name"] != "transfer_data":
                    _alloc[task["id"]] = task["params"]["agent"]
            self.alloc = _alloc
            self.pub_rac.publish(json.dumps(self.alloc))
        else:
            self.logwarn("RAC not ready")


if __name__ == "__main__":
    # Initialize_node
    rospy.init_node('rac', anonymous=True)

    aid = rospy.get_param('~agent_id')

    # Create an instance of the RAC
    #rac = example_resource_allocation_matcher(aid)
    rac = img_nav_drive_resource_allocation_matcher(aid)

    # static allocation from a param
    #alloc = rospy.get_param('~alloc',{})
    #rac = static_resource_allocation_matcher(aid,alloc)

    # Run it
    rac.run()
