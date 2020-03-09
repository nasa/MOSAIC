#!/usr/bin/python

'''
# =====================================================================
# Obligation handlers for ROS actions and services
#
# Author: Marc Sanchez Net
# Date:   02/06/2019
# Copyright (c) 2019, Jet Propulsion Laboratory.
# =====================================================================
'''

# Generic imports
import abc
import json
import time
from threading import Lock

# ROS imports
import rospy
import actionlib

# PDRA imports
from pdra.core import Obligation, Result

# =====================================================================
# === Global variables
# =====================================================================

# Rate at which action state is checked [sec]
RATE = 1

# =====================================================================
# === Obligation Handler for ROS Action
# =====================================================================


class OhdlrState(object):
    def __init__(self):
        # Can be "empty", "pending", "completed"
        self.status = None

        # Can be None, UID, Result
        self.value = None

        # The state of the ohdlr might be modified by multiple threads,
        # so use a lock to ensure no race conditions (only for write operations)
        self._lock = Lock()

    @property
    def is_pending(self):
        return self.status == 'pending'

    @property
    def is_empty(self):
        return self.status == 'empty'

    @property
    def has_result(self):
        return self.status == 'completed'

    def reset(self):
        with self._lock:
            self.status = 'empty'
            self.value = None

    def set_pending(self, uid):
        with self._lock:
            self.status = 'pending'
            self.value = uid

    def set_result(self, result):
        with self._lock:
            self.status = 'completed'
            self.value = result.values

    def __str__(self):
        return '<State [{}|{}}]>'.format(self.status, self.value)


class PdraObligationHandlerAction(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, dispatcher, agent_id, resource_id, timeout=float('inf')):
        # Store variables
        self.dispatcher = dispatcher
        self.agent_id = agent_id
        self.resource_id = resource_id
        self.timeout = timeout
        self._result = None
        self.is_alive = True

        # Create a variable for the state of this ohdlr.
        self.state = OhdlrState()

        # Define action server and start it
        self._as = actionlib.SimpleActionServer(resource_id, self.action_type_cls,
                                                execute_cb=self._new_goal,
                                                auto_start=False)
        self._as.start()

    @abc.abstractproperty
    def action_type_cls(self):
        pass

    @abc.abstractproperty
    def action_result_cls(self):
        pass

    @abc.abstractproperty
    def action_feedback_cls(self):
        pass

    @abc.abstractmethod
    def serialize_goal_request(self, goal):
        pass

    def deserialize_goal_dependents(self, goal):
        # Try to deserialize the list of dependent obligations
        # If it fails with an attribute error, then this obligation
        # does not have dependents
        try:
            return json.loads(goal.dependents)
        except AttributeError:
            return None

    def _shutdown(self):
        self.is_alive = False

    def _new_goal(self, goal):
        """ Handle receiving an Obligation from the upper layer """
        # Update log
        self.loginfo('Received request for ' + goal.request)
        # Initialize variables
        success = False
        rate = rospy.Rate(RATE)

        # Create obligation from goal
        obl = Obligation(resource_id=self.resource_id,
                         req_agent=self.agent_id,
                         TTL=self.timeout,
                         parameters=self.serialize_goal_request(goal),
                         dependents=self.deserialize_goal_dependents(goal))

        # Indicate that a result is pending
        self.state.set_pending(obl.uid)

        # Publish the obligation
        self.dispatcher._new_obligation(obl)

        # Wait for the obligation result
        while not success and self.is_alive:
            # If rospy died, stop immediately
            if rospy.is_shutdown():
                self._goal_failed()
                break

            # If state is empty, something went wrong, it shouldn't happen
            if self.state.is_empty:
                self._goal_failed()
                self.logerr('Empty state in the Obligation Handler')
                break

            # If the timeout expires, then the obligation is no longer valid
            if not obl.valid:
                self._goal_failed()
                break

            # If the client has preempted the previous task, return
            if self._as.is_preempt_requested():
                self._goal_preempted()
                break

            # If you still don't have results, keep waiting
            if self.state.is_pending:
                self._goal_pending(obl)
                rate.sleep()
                continue

            # If you don't have a result by now, error
            if not self.state.has_result:
                self._goal_failed()
                self.logerr(
                    'Obligation Handler state should have result by now')
                break

            # If you reach this point, you have a result waiting for you
            self._goal_succeeded()
            success = True

    def _new_result(self, result):
        """ Handle receiving a Result from the Acceptor """
        # If you don't have a result pending, return
        # (e.g. the goal has timed out)
        if not self.state.is_pending:
            return

        # If this result is not for the pending obligation, return
        # (e.g. result for an action that was preempted)
        if self.state.value != result.uid:
            return

        # If this result was not processed by the same node, flag
        if result.req_agent != result.srv_agent:
            self.loginfo('Received result computed in {}', result.srv_agent)
        else:
            self.loginfo('Received result computed locally')

        # Store the result in the state
        self.state.set_result(result)

    def _goal_succeeded(self):
        # Update log
        self.loginfo('Success')

        # Create the result
        result = self.action_result_cls()
        result.result = self.state.value

        # Indicate success in actionlib
        self._as.set_succeeded(result)

        # Reset state
        self.state.reset()

    def _goal_preempted(self):
        # Update log
        self.loginfo('Preempted')

        # Create the result
        result = self.action_result_cls()
        result.result = 'preempted'

        # Indicate preemption
        self._as.set_preempted(result, 'Goal preempted')

        # Reset the state
        self.state.reset()

    def _goal_failed(self):
        # Update log
        self.loginfo('Failed')

        # Create the result
        result = self.action_result_cls()
        result.result = 'failed'

        # Indicate failure in actionlib
        self._as.set_aborted(result, 'Goal failed')

        # Reset the state
        self.state.reset()

    def _goal_pending(self, obl):
        # Update log
        #self.loginfo('Awaiting task completion ' + obl.parameters)
        pass

    def logdebug(self, msg, *args):
        rospy.logdebug('[%s/%s_ohdlr]: %s', self.agent_id,
                       self.resource_id, msg.format(*args))

    def loginfo(self, msg, *args):
        rospy.loginfo('[%s/%s_ohdlr]: %s', self.agent_id,
                      self.resource_id, msg.format(*args))

    def logerr(self, msg, *args):
        rospy.logerr('[%s/%s_ohdlr]: %s', self.agent_id,
                     self.resource_id, msg.format(*args))

    def logwarn(self, msg, *args):
        rospy.logwarn('[%s/%s_ohdlr]: %s', self.agent_id,
                      self.resource_id, msg.format(*args))


# =====================================================================
# === Obligation Handler for ROS Service
# =====================================================================

# TODO
