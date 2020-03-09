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
from std_msgs.msg import String
from pdra.dispatcher import Dispatcher, start_dispatcher
import rospy
import json
from defaults_scaffolding import patch_imports
patch_imports()

# Generic imports

# ROS imports

# PDRA imports

# =====================================================================
# === Test Dispatcher
# =====================================================================


class TestDispatcher(Dispatcher):

    def __init__(self, *args, **kwargs):
        super(TestDispatcher, self).__init__(*args, **kwargs)
        rospy.Subscriber('rac/output', String, self.process_rac_output)

        # Dictionary of resource-task allocations. Start with a default value
        self.alloc = {
            'img': 'node_92',
            'nav': 'node_92',
            'drive': 'node_92',
            'process': 'node_91',
            'store': 'base_station'
        }

    def where_to_process(self, dsp):
        # Create resource ids
        short_res_id = dsp.resource_id
        long_res_id = '{}:{}'.format(dsp.req_agent, dsp.resource_id)
        # HACK: tasks for samples start with a number
        sample_res_id = '{}:0{}'.format(dsp.req_agent, dsp.resource_id)

        # Use short ID if possible
        try:
            return self.alloc[short_res_id]
        except KeyError:
            pass

        # Use long ID if possible
        try:
            return self.alloc[long_res_id]
        except KeyError:
            pass

        # HACK: Use sample resource ID if possible
        try:
            return self.alloc[sample_res_id]
        except KeyError:
            pass

        return "UNKNOWN"

    def process_rac_output(self, msg):
        # Cache the activity-resource allocation map
        _new_alloc = json.loads(msg.data)
        if self.alloc != _new_alloc:
            rospy.loginfo(
                "[{}/Dispatcher]: Received new RAC map".format(self.agent_id))
            self.alloc = json.loads(msg.data)

# =====================================================================
# === MAIN
# =====================================================================


if __name__ == '__main__':
    # Start ROS node
    rospy.init_node('dispatcher_test', anonymous=True)

    # Start dispatcher
    start_dispatcher(TestDispatcher)
