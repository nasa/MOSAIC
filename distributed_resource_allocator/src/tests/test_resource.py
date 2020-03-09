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
z
"""

import rospy
import math
import yaml
import uuid

import actionlib
from std_msgs.msg import String

from pdra.msg import RequestResourceFeedback, RequestResourceResult, RequestResourceAction


class ResourceServer(object):
    """The Resource Server Class

    """

    # create messages that are used to publish feedback/result
    _feedback = RequestResourceFeedback()
    _result = RequestResourceResult()

    def __init__(self, name, agent_id='', duration=1):
        self.action_name = name
        self.agent_id = agent_id
        self.duration = duration
        # Action server
        self.action_server = actionlib.SimpleActionServer(
            self.action_name, RequestResourceAction, execute_cb=self.execute_request, auto_start=False)
        self.action_server.start()

        self.loginfo('resource started')

    def execute_request(self, goal):
        """Handle resource requests"""

        # start executing the activity
        action_start_time = rospy.Time.now().to_sec()

        # helper variables
        success = False
        acitivity_finished = False
        r = rospy.Rate(1)  # 10hz

        # start executing the activity
        while not acitivity_finished:
            # check that preempt has not been requested by the client
            if self.action_server.is_preempt_requested():
                self.loginfo('Preempted')
                self._result.result = 'Preempted'
                self.action_server.set_preempted(
                    self._result, 'Resource request preempted')
                return

            elapsed_time = rospy.Time.now().to_sec() - action_start_time

            # publish the feedback
            progress = (elapsed_time/self.duration)*100.0
            if progress > 100:
                progress = 100
            self._feedback.progress = int(progress)
            self.action_server.publish_feedback(self._feedback)

            # check if the activity is done
            if elapsed_time >= self.duration:
                acitivity_finished = True
                success = True
                break

            r.sleep()

        if success:
            self.loginfo('Succeeded')
            self._result.result = 'Succeeded'
            self.action_server.set_succeeded(self._result)
        else:
            self.loginfo('Failed')
            self._result.result = 'Succeeded'
            self.action_server.set_aborted(self._result)

    def loginfo(self, msg, *args):
        msg = msg.format(*args)
        rospy.loginfo('[%s/%s]: %s', self.agent_id, self.action_name, msg)


if __name__ == '__main__':
    rospy.init_node('resource')

    agent_id = rospy.get_param('~agent_id')
    resource_id = rospy.get_param('~resource_id', rospy.get_name())
    agent_id = rospy.get_param('~agent_id', '')
    duration = float(rospy.get_param('~duration', 5))

    server = ResourceServer(resource_id, agent_id, duration)
    # Spin forever
    rospy.spin()
