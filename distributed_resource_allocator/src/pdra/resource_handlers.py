#!/usr/bin/python

'''
# =====================================================================
# Abstract resource handlers for ROS Actions and ROS Services
#
# .. Warning:: ROS Services are currently not supported
#
# Author: Marc Sanchez Net
# Date:   05/15/2019
# Copyright (c) 2019, Jet Propulsion Laboratory.
# =====================================================================
'''

# General imports
import abc
from threading import Lock, Semaphore, Thread
from Queue import PriorityQueue
import json

# ROS import
from actionlib import SimpleActionClient
import rospy
from std_msgs.msg import String

# PDRA imports
from pdra.core import Result

# =====================================================================
# === Global variables
# =====================================================================

# Max size for the dispatcher queues. 0 = infinity
MAX_QUEUE_SIZE = 0

# =====================================================================
# === Abstract Resource Handler for ROS Actions
# =====================================================================


class PdraResourceHandlerAction(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, dispatcher, agent_id, resource_id, preempt_actions=False):
        # Store variables
        self.dispatcher = dispatcher
        self.agent_id = agent_id
        self.resource_id = resource_id
        self.current_obligation = None
        self.preempt_actions = preempt_actions
        self.is_alive = True
        self.status_publisher = rospy.Publisher(
            'resources_in_execution', String, queue_size=10)
        self.status_log_publisher = rospy.Publisher(
            'resources_activity_log', String, queue_size=10)

        # Create a queue to store new obligations as they arrive
        self._queue = PriorityQueue(maxsize=MAX_QUEUE_SIZE)

        # Create a semaphore to avoid action preemption of ROS actionlib
        self.semaphore = Semaphore(0)

        # Create a lock for the current obligation
        self.lock = Lock()

        # Create client to request services
        ac_name = self.action_name()
        self.ac = SimpleActionClient(ac_name, self.action_type_cls)
        self.ac.wait_for_server()

        # Run handler
        self.run()

    def action_name(self):
        return '/{}/resource/{}'.format(self.agent_id, self.resource_id)

    @abc.abstractproperty
    def action_type_cls(self):
        pass

    @abc.abstractproperty
    def action_goal_cls(self):
        pass

    @abc.abstractmethod
    def action_feedback_cls(self):
        pass

    @abc.abstractmethod
    def fill_goal_info(self, *args, **kwargs):
        pass

    @abc.abstractmethod
    def serialize_goal_results(self, *args, **kwargs):
        pass

    def log_request_done(self, *args, **kwargs):
        pass

    def run(self):
        self.th = Thread(target=self._process_obligations)
        self.th.setDaemon(True)
        self.th.start()

    def _shutdown(self):
        """ Indicate exit with max priority """
        self.is_alive = False
        self._queue.put_nowait((-float('inf'), ''))

    def _new_obligation(self, obl):
        """ Put new obligations into to the processing queue """
        # Store in queue
        self._queue.put_nowait((obl.priority, obl))

        # Log obligation arrival
        self.loginfo('Queued obligation from agent {}. Queue length: {}'.format(
            obl.req_agent, self._queue.qsize()))

    def _process_obligations(self):
        # Process forever
        while True:
            # Get the next obligation. If empty, block
            priority, obl = self._queue.get(True)

            # If no longer alive, exit
            if not self.is_alive:
                break

            # If this obligation is not valid, skip
            if not obl.valid:
                self.loginfo('Invalid obligation.')
                continue

            # Log arrival
            self.loginfo('Process obligation from agent {}. Queue length: {}'.format(
                obl.req_agent, self._queue.qsize()))
            res_status = (obl.req_agent, self.resource_id,
                          'start', rospy.get_time())
            self.status_publisher.publish(json.dumps(res_status))

            # Protect everything with a lock to avoid race conditions
            with self.lock:
                # Store current obligation
                self.current_obligation = obl

                # Create the new goal
                goal = self.action_goal_cls()

                # Add parameters to goal
                goal = self.fill_goal_info(goal)

                # Send out obligation to resource
                self.ac.send_goal(goal, done_cb=self.request_done,
                                  feedback_cb=self.request_feedback)

            # If you want to preempt actions, continue
            if self.preempt_actions:
                continue

            # Wait until the action has finished if no preemption is desired
            self.semaphore.acquire()

    def request_done(self, goal_status, goal_result):
        """ Handles request completion """
        # Update log
        self.loginfo('Request done.')
        res_status = (self.current_obligation.req_agent,
                      self.resource_id, 'end', rospy.get_time())

        self.status_publisher.publish(json.dumps(res_status))

        self.log_request_done(goal_result)

        # Create a new result
        result = Result.from_obligation(self.current_obligation,
                                        srv_agent=self.agent_id,
                                        values=self.serialize_goal_results(goal_result))

        # Delete current obligation. Not needed anymore
        with self.lock:
            self.current_obligation = None

        # Pass the new result to the dispatcher
        self.dispatcher._new_result(result)

        # If you are not managing preemption, you are done
        if self.preempt_actions:
            return

        # Indicate that the action has finished
        self.semaphore.release()

    def request_feedback(self, msg):
        """ Handles request feedback """
        pass

    def loginfo(self, msg, *args):
        msg = msg.format(*args)
        rospy.loginfo('[%s/%s_rhdlr]: %s', self.agent_id,
                      self.resource_id, msg)

    def logwarn(self, msg, *args):
        msg = msg.format(*args)
        rospy.logwarn('[%s/%s_rhdlr]: %s', self.agent_id,
                      self.resource_id, msg)

    def logerr(self, msg, *args):
        msg = msg.format(*args)
        rospy.logerr('[%s/%s_rhdlr]: %s', self.agent_id,
                     self.resource_id, msg)

    def logdebug(self, msg, *args):
        msg = msg.format(*args)
        rospy.logdebug('[%s/%s_rhdlr]: %s', self.agent_id,
                       self.resource_id, msg)

# =====================================================================
# === Abstract Resource Handler for ROS Services
# =====================================================================

# TODO
