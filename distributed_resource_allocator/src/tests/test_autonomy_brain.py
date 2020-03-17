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

import time
import json

import actionlib
import rospy
from std_msgs.msg import String

import defaults_scaffolding as def_vals
import pdra.msg as msg


class AutonomyBrain(object):
    def __init__(self, agent_id, activity_names, chained_activities=None):
        # Store variables
        self.agent_id = agent_id

        self.plan = None
        self.activity_index = -1
        self.start_time = time.time()
        self.ongoing_activity = None

        # Create subscriber
        rospy.Subscriber('/all/autonomy/activity_plans',
                         String, self.receive_plan)

        # Map of {action name: SimpleActionClient} for simple actions
        self.activity_clients = {}

        # Map of {action name: SimpleActionClient} for chained actions
        self.chained_clients = {}

        # Create an action client for each simple action (without chaining)
        self.activitity_names = activity_names
        for activity_name in self.activitity_names:
            path = '/{}/distware/{}'.format(self.agent_id, activity_name)
            ac = actionlib.SimpleActionClient(path, msg.RequestResourceAction)
            self.activity_clients[activity_name] = ac

        # Create an action client for a chained action
        self.chained_activities = chained_activities
        for chained_act in self.chained_activities:
            path = '/{}/distware/{}'.format(self.agent_id, chained_act)
            ac = actionlib.SimpleActionClient(path, msg.RequestChainAction)
            self.chained_clients[chained_act] = ac

        # Wait a little bit
        time.sleep(10)

    def receive_plan(self, msg):
        """ Receives new plan to execute"""
        # From string to dictionary
        activity_plans = eval(msg.data)

        # If this plan is not for this agent, skip
        if self.agent_id not in activity_plans['Plans']:
            return

        # Get the tasks for this agent
        agent_plan = activity_plans['Plans'][self.agent_id]['tasks']

        # If empty agent plan, skip
        if not agent_plan:
            return

        # if the current plan is equal to the received plan, skip
        if self.plan == agent_plan:
            return

        # Update the activity plan for the agent
        self.plan = agent_plan
        self.activity_index = -1
        self.loginfo("received new plan {}", self.plan)

    def dispatch_activity(self):
        """ Check if an activity has to be dispatch"""
        activity_to_dispatch = None

        # We are doing things sequential so if there is an activity that hasn't finished yet
        # it will wait for its completion.
        current_time = time.time() - self.start_time

        # if plan is empty, return
        if not self.plan:
            return

        # If there is an ongoing activity, you can't dispatch a new one
        if self.ongoing_activity:
            return

        # if the plan execution hasn't started (or has finished), pick the first activity in the plan
        if self.activity_index == -1:
            activity_to_dispatch = self.plan[0]
            self.activity_index = 0
            self.start_time = time.time()
            current_time = 0
        elif self.activity_index == (len(self.plan) - 1):
            # if we finish executing the last activity, restart from teh first activity
            last_activity = self.plan[self.activity_index]
            plan_end_time = last_activity['start_time'] + \
                last_activity['duration']
            if current_time >= plan_end_time:
                activity_to_dispatch = self.plan[0]
                self.activity_index = 0
                self.start_time = time.time()
                current_time = 0
        else:
            # if no other activity is running run then check next activity to dispatch
            for i in range(self.activity_index + 1, len(self.plan)):
                activity = self.plan[i]
                activity_start_time = activity['start_time']
                if current_time >= activity_start_time:
                    self.activity_index += 1
                    activity_to_dispatch = activity
                    break

        # If you don't have an activity to dispatch, return
        if activity_to_dispatch is None:
            return

        # Update log
        self.loginfo(activity_to_dispatch['name'] + ' at ' + str(current_time))

        # Mark the ongoing activity
        self.ongoing_activity = activity_to_dispatch

        # Try to send the goal as a simple action. If it fails, it is a chained action
        try:
            self.request_simple_action()
        except KeyError:
            pass
        else:
            return

        # Try to send the goal as a chained action.
        try:
            self.request_chained_action()
        except KeyError:
            pass
        else:
            return

        # If you reach this point, you are dealing with an unknown action. Show message
        # and reset the ongoing activity
        rospy.logerr('Unknown action type {}'.format(activity_to_dispatch))
        self.ongoing_activity = None

    def request_simple_action(self):
        # Get action client for this type of activity
        client = self.activity_clients[self.ongoing_activity['name']]

        # Build the goal
        goal = msg.RequestResourceGoal()
        goal.request = self.ongoing_activity['name']

        # Send the goal
        client.send_goal(goal, done_cb=self.request_done,
                         feedback_cb=self.request_feedback)

    def request_chained_action(self):
        # Get action client for this type of activity
        client = self.chained_clients[self.ongoing_activity['name']]

        # Build the goal
        goal = msg.RequestChainGoal()
        goal.request = self.ongoing_activity['name']
        goal.dependents = json.dumps(
            self.ongoing_activity['params']['dependents'])

        # Send the goal
        client.send_goal(goal, done_cb=self.request_done,
                         feedback_cb=self.request_feedback)

    def request_done(self, status, msg):
        """ Handles request completion """
        self.loginfo(
            'Activity {} DONE.'.format(self.ongoing_activity['name']))
        self.ongoing_activity = None

    def request_feedback(self, msg):
        """ Handles request feedback """
        pass

    def loginfo(self, msg, *args):
        msg = msg.format(*args)
        rospy.loginfo('[%s/autonomy_brain]: %s', self.agent_id, msg)


if __name__ == "__main__":
    # Start ROS node
    rospy.init_node('autonomy_brain_test', anonymous=True)

    # Parse node inputs
    rate = rospy.get_param('~rate', def_vals.PUB_RATE)
    agent_id = rospy.get_param('~agent_id')
    resource_ids = eval(rospy.get_param('~resource_ids', ''))

    # Get the resources you want to access
    res_ids = resource_ids['basic']
    res_chd = resource_ids.get('chained', [])

    # Create autonomy brain
    autonomy_brain = AutonomyBrain(agent_id, res_ids, res_chd)

    # Rate at which world plan is published
    rate_ctrl = rospy.Rate(rate)

    # Do until ROS finishes
    while not rospy.is_shutdown():
        autonomy_brain.dispatch_activity()
        rate_ctrl.sleep()
