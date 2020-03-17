#!/usr/bin/python

'''
# =====================================================================
# Dispatcher for testing purposes
#
# Author: Marc Sanchez Net
# Date:   05/15/2019
# Copyright (c) 2019, Jet Propulsion Laboratory.
# =====================================================================
'''

# Generic imports
import abc
import json
import traceback

# ROS imports
import rospy

# PDRA imports
from pdra.core import Obligation, Result
from pdra.utils import now

# =====================================================================
# === Abstract Radio
# =====================================================================


class PdraRadio(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, agent_id, parent, *args, **kwargs):
        # Store variables
        self.agent_id = agent_id
        self.parent = parent        # Comm handler object

    @abc.abstractmethod
    def send(self, *args, **kwargs):
        pass

    @abc.abstractmethod
    def receive(self, *args, **kwargs):
        pass

    def run(self):
        pass

# =====================================================================
# === Abstract Comm Handler
# =====================================================================


class PdraCommHandler(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, dispatcher, agent_id, resource_id, *args, **kwargs):
        # Store variables
        self.dispatcher = dispatcher
        self.agent_id = agent_id
        self.resource_id = resource_id

        # Create a radio object
        self.radio = self.new_radio(**kwargs)

        # Run the radio
        self.radio.run()

    @abc.abstractmethod
    def new_radio(self, **kwargs):
        pass

    def _send_dispatchable(self, orig, dest, dsp):
        """ Process dispatchable arriving from the forwarder """
        try:
            self.radio.send(orig, dest, dsp.to_json(), TTL=dsp.TTL)
        except:
            self.logerr(traceback.format_exc())

    def receive_dispatchable(self, dsp):
        """ Process dispatchable arriving from the radio/comm channels """
        # Deserialize dispatchable from the radio
        dsp = self._deserialize_dispatchable(dsp)

        # If the dispatchable could not be deserialized, return
        if dsp is None:
            self.logerr('Received dispatchable that cannot be deserialized')
            return

        # If the dispatchable is not valid, skip
        if not dsp.valid:
            self.logerr('Received invalid dispatchable:\n Requesting agent is {}\nType is {}\nResource ID is {}\n TTL is {}, creation time is {}, and now is {} (remaining TTL {})'.format(
                dsp.req_agent, dsp.type, dsp.resource_id, dsp.TTL, dsp.creation_time, now(), -(now()-dsp.creation_time) + dsp.TTL))
            self.logerr("srv_agent: {}, uid: {}, resource_id: {}, req_agent: {}, creation_time: {}, priority: {}, version: {}".format(
                dsp.srv_agent,
                dsp.uid,
                dsp.resource_id,
                dsp.req_agent,
                dsp.creation_time,
                dsp.priority,
                dsp.version,
                # dsp.values[:100],
            ))
            return

        # This is a result. Send it to the acceptor
        self.dispatcher._new_result(dsp)

    def _deserialize_dispatchable(self, json_dsp):
        # Figure out which type of dispatchable it is
        dsp_type = json.loads(json_dsp)['type'].lower()

        # Handle a new obligation
        if dsp_type == 'obligation':
            return Obligation.from_json(json_dsp)

        # Handle a new result
        if dsp_type == 'result':
            return Result.from_json(json_dsp)

        # If you reach this point, error
        self.logerr('Unknown dispatchable received')
        self.logerr(json_dsp)

    def logdebug(self, msg, *args):
        rospy.logdebug('[%s/%s_chdlr]: %s', self.agent_id,
                       self.resource_id, msg.format(*args))

    def loginfo(self, msg, *args):
        rospy.loginfo('[%s/%s_chdlr]: %s', self.agent_id,
                      self.resource_id, msg.format(*args))

    def logerr(self, msg, *args):
        rospy.logerr('[%s/%s_chdlr]: %s', self.agent_id,
                     self.resource_id, msg.format(*args))

    def logwarn(self, msg, *args):
        rospy.logwarn('[%s/%s_chdlr]: %s', self.agent_id,
                      self.resource_id, msg.format(*args))
