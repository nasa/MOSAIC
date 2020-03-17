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

# Patch imports
import defaults_scaffolding as def_vals
from pdra.msg import CommData
from pdra.comm_handlers import PdraCommHandler, PdraRadio
from std_msgs.msg import String
import rospy
from defaults_scaffolding import patch_imports
patch_imports()

# ROS imports

# PDRA imports

# =====================================================================
# === Define Radio
# =====================================================================


class BroadcastRadio(PdraRadio):
    def __init__(self, agent_id, parent, **kwargs):
        # Call parent constructor
        super(BroadcastRadio, self).__init__(agent_id, parent)

        # Create publisher to the comm channel
        self.pub = rospy.Publisher('/comm/channel', CommData,
                                   queue_size=def_vals.PUB_QUEUE_SIZE)

        # Create subscriber to the comm channel
        rospy.Subscriber('/comm/channel', CommData, self.receive)

    def send(self, orig, dest, dsp, **kwargs):
        # Publish message
        self.pub.publish(receiver=dest, data=dsp)

    def receive(self, msg):
        # If this message is not for this agent, skip
        if msg.receiver != self.agent_id:
            return

        # Receive the dispatchable
        self.parent.receive_dispatchable(msg.data)


class SimpleDTNRadio(PdraRadio):
    def __init__(self, agent_id, parent, **kwargs):
        # Call parent constructor
        super(SimpleDTNRadio, self).__init__(agent_id, parent)

        # Create publisher to the comm channel
        self.pub = rospy.Publisher('/{}/comm/tx'.format(self.agent_id), CommData,
                                   queue_size=def_vals.PUB_QUEUE_SIZE)

        # Create subscriber to the comm channel
        rospy.Subscriber('/{}/comm/rx'.format(self.agent_id),
                         CommData, self.receive)

    def send(self, orig, dest, dsp, **kwargs):
        # Publish message
        self.pub.publish(receiver=dest, data=dsp)

    def receive(self, msg):
        # If this message is not for this agent, skip
        if msg.receiver != self.agent_id:
            return

        # Receive the dispatchable
        self.parent.receive_dispatchable(msg.data)


# =====================================================================
# === Define Comm handler
# =====================================================================


class TestCommHandler(PdraCommHandler):
    def new_radio(self, **kwargs):
        return BroadcastRadio(self.agent_id, self, **kwargs)


class TestSimpleDTNCommHandler(PdraCommHandler):
    def new_radio(self, **kwargs):
        return SimpleDTNRadio(self.agent_id, self, **kwargs)
