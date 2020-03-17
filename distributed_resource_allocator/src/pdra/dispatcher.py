#!/usr/bin/python

'''
# =====================================================================
# Abstract PDRA dispatcher class
#
# Author: Marc Sanchez Net
# Date:   01/29/2019
# Copyright (c) 2019, Jet Propulsion Laboratory.
# =====================================================================
'''

# General imports
import abc
import ast
from Queue import PriorityQueue
from threading import Thread
import threading

# ROS imports
import rospy

# PDRA imports
from pdra.core import Obligation, Result
from pdra.utils import load_class_dynamically, now

# =====================================================================
# === Global variables
# =====================================================================

# Max size for the dispatcher queues. 0 = infinity
MAX_QUEUE_SIZE = 0
PUB_QUEUE_SIZE = 100

# Default timeout for a resource
RES_TIMEOUT = 600

# =====================================================================
# === Dispatchable Implementation
# =====================================================================


class Dispatcher(object):
    """ Abstract dispatcher class.

        :ivar string: agent_id
        :ivar PriorityQueue: _up_queue
        :ivar PriorityQueue: _dn_queue
        :ivar dict: ohandlers
        :ivar dict: rhandlers
        :ivar object: chandler
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, agent_id):
        """ Class constructor

            :param string: Agent ID
        """
        # Store variables
        self.agent_id = agent_id

        # Incoming queue (from back-end up to front-end)
        self._up_queue = PriorityQueue(maxsize=MAX_QUEUE_SIZE)

        # Outgoing queue (from front-end to back-end)
        self._dn_queue = PriorityQueue(maxsize=MAX_QUEUE_SIZE)

        # Communication Handler
        self.chandler = None

        # Dictionary {resource_id: obligation_handler object}
        self.ohandlers = {}

        # Dictionary {resource_id: resource_handler object}
        self.rhandlers = {}

        # Exit all threads upon rospy shutdown
        rospy.on_shutdown(self._shutdown_dispatcher)

        # Run the dispatcher and acceptor
        self.run()

    @abc.abstractmethod
    def where_to_process(self, *args, **kwargs):
        pass

    @abc.abstractmethod
    def process_rac_output(self, *args, **kwargs):
        pass

    def run(self):
        # Start forwarder in a separate thread
        self.th_fwdr = Thread(target=self._run_forwarder)
        self.th_fwdr.setDaemon(True)
        self.th_fwdr.start()

        # Start acceptor in a separate thread
        self.th_acptr = Thread(target=self._run_acceptor)
        self.th_acptr.setDaemon(True)
        self.th_acptr.start()

    def _shutdown_dispatcher(self):
        # Signal the forwarder and acceptor that they must exit
        self._up_queue.put_nowait((-float('inf'), 'exit'))
        self._dn_queue.put_nowait((-float('inf'), 'exit'))

        # Stop all obligation handlers
        for oh in self.ohandlers.values():
            oh._shutdown()

        # Stop all resource handlers
        for rh in self.rhandlers.values():
            rh._shutdown()

    def _run_forwarder(self):
        """ The forwarder pulls from ``self._dn_queue`` and directs the
            dispatchable depending on the RAC input
        """
        while True:
            # Get the next dispatchable. If empty, block
            priority, dsp = self._dn_queue.get(True)

            # If dsp signals exit, you are done
            if dsp == 'exit':
                break

            # Check if this dispatchable is valid. If not, log error
            if not dsp.valid:
                self._logerr('[Forwarder] Invalid dispatchable:\n Requesting agent is {}\nType is {}\nResource ID is {}\n TTL is {}, creation time is {}, and now is {} (remaining TTL {})'.format(
                    dsp.req_agent, dsp.type, dsp.resource_id, dsp.TTL, dsp.creation_time, now(), -(now()-dsp.creation_time) + dsp.TTL))
                self._logerr("srv_agent: {}, uid: {}, resource_id: {}, req_agent: {}, creation_time: {}, priority: {}, version: {}".format(
                    dsp.srv_agent,
                    dsp.uid,
                    dsp.resource_id,
                    dsp.req_agent,
                    dsp.creation_time,
                    dsp.priority,
                    dsp.version,
                    # dsp.values[:100],
                ))
                continue

            # If this dispatchable is a result, send it through the comm handler
            if dsp.type == 'Result':
                self.chandler._send_dispatchable(
                    dsp.srv_agent, dsp.req_agent, dsp)
                continue

            # If this dispatchable is an obligation, get the agent that must
            # process it
            srv_agent = self.where_to_process(dsp)

            # If the serving agent is not the local agent, send it
            if srv_agent not in (None, "UNKNOWN", self.agent_id):
                self.chandler._send_dispatchable(dsp.req_agent, srv_agent, dsp)
                continue

            # If srv_agent is "None", the RAC thinks this should not be done.
            # Create a crude result and bounce.
            # TODO: we do not know the values expected by the obligation handler,
            # since we have no idea of its internal format. So we just write our
            # displeasure all over the serial stream.
            if srv_agent is None:
                # self._logerr('Resource {} not known by RAC. Attempting to satisfy locally'.format(
                    # dsp.resource_id))
                self._logerr('Resource {} should not be executed according to RAC. Skipping!'.format(
                    dsp.resource_id))
                bounced_result = Result.from_obligation(dsp,
                                                        srv_agent=self.agent_id,
                                                        values="BOUNCED")
                self._new_result(bounced_result)
                continue

            # If srv_agent is "UNKNOWN", show message that you will default to local execution.
            if srv_agent == "UNKNOWN":
                self._logerr('Resource {} not known by RAC. Attempting to satisfy locally'.format(
                    dsp.resource_id))

            # Send a new obligation to a local resource. If it is not available, just
            # abandon it for now. TODO: Trigger some cancellations
            try:
                self.rhandlers[dsp.resource_id]._new_obligation(dsp)
            except KeyError:
                self._logerr(
                    'Resource {} is not available locally'.format(dsp.resource_id))
                self._logerr('Resources available are {}'.format(
                    self.rhandlers.keys()))

    def _run_acceptor(self):
        """ The acceptor pulls from ``self._up_queue`` and directs the result
            depending on whether it is local or remote.
        """
        while True:
            # Get the next result. If empty, block
            priority, dsp = self._up_queue.get(True)

            # If res signals exit, you are done
            if dsp == 'exit':
                break

            # Check if this dispatchable is valid. If not, log error
            if not dsp.valid:
                self._logerr('Invalid result at acceptor:\n Requesting agent is {}\nType is {}\nResource ID is {}\n TTL is {}, creation time is {}, and now is {} (remaining TTL {})'.format(
                    dsp.req_agent, dsp.type, dsp.resource_id, dsp.TTL, dsp.creation_time, now(), -(now()-dsp.creation_time) + dsp.TTL))
                self._logerr("srv_agent: {}, uid: {}, resource_id: {}, req_agent: {}, creation_time: {}, priority: {}, version: {}".format(
                    dsp.srv_agent,
                    dsp.uid,
                    dsp.resource_id,
                    dsp.req_agent,
                    dsp.creation_time,
                    dsp.priority,
                    dsp.version,
                    # dsp.values[:100],
                ))

                # self._logerr(dsp.to_json())
                continue

            # If this is an obligation coming from the comm handler, pass it to
            # forwarder directly
            if dsp.type == 'Obligation':
                self._new_obligation(dsp)
                continue

            # If this result has dependents, create a dependent obligation and
            # pass it to the forwarder for processing
            if dsp.dependents is not None:
                self._create_dependent(dsp)
                continue

            # If this result's requesting agent is not this agent, pass to forwarder
            # so that it sends it using the comm handler
            if dsp.req_agent != self.agent_id:
                self._new_obligation(dsp)
                continue

            # If you reach this point, this result is for this agent. To see which resource
            # triggered this (potential chain of) result(s), look at the predecessors
            if not any(dsp.predecessors):
                resource_id = dsp.resource_id
            else:
                resource_id = dsp.predecessors[0]['resource_id']

            # Send the result to the appropriate obligation handler
            try:
                self.ohandlers[resource_id]._new_result(dsp)
            except KeyError:
                self._logerr(
                    'Obligation handler for resource {} is not available'.format(resource_id))
                self._logerr('Obligation handlers are {}'.format(
                    self.ohandlers.keys()))

    def _create_dependent(self, result):
        self._loginfo('Creating dependent obligation from {}'.format(result))

        # Get the next dependent resource
        resource_id = result.dependents.pop(0)

        # If list of dependents of empty, reset to None
        if not any(result.dependents):
            result.dependents = None

        # Get the parameters of the dependent obligation as the values
        # from the current result
        params = result.values

        # Create new obligation
        obl = Obligation.from_result(result, resource_id, params)

        # Enqueue new obligation for processing
        self._new_obligation(obl)

    def _new_obligation(self, dsp):
        # Put the obligation in the outgoing queue to process
        self._dn_queue.put_nowait((dsp.priority, dsp))

    def _new_result(self, res):
        # Put the result in the incoming queue to process
        self._up_queue.put_nowait((res.priority, res))

    def _add_obligation_handler(self, oh_id, oh_type, directory='.', **kwargs):
        # Get handler class
        self.ohandlers[oh_id] = self._new_handler(
            oh_id, oh_type, directory, **kwargs)

    def _add_resource_handler(self, rh_id, rh_type, directory='.', **kwargs):
        # Store new handler
        self.rhandlers[rh_id] = self._new_handler(
            rh_id, rh_type, directory, **kwargs)

    def _add_comm_handler(self, ch_id, ch_type, directory='.', **kwargs):
        # Store new handler
        self.chandler = self._new_handler(ch_id, ch_type, directory, **kwargs)

    def _new_handler(self, resource_id, handler_type, directory, **kwargs):
        # Separate module and class name
        module, class_name = handler_type.split('.')

        # Load the class definition to be created
        cls = load_class_dynamically(directory, module, class_name=class_name)

        # Create the obligation handler object
        return cls(self, self.agent_id, resource_id, **kwargs)

    def _logerr(self, msg):
        rospy.logerr("[{}/dispatcher]: {}".format(self.agent_id, msg))

    def _logwarn(self, msg):
        rospy.logwarn("[{}/dispatcher]: {}".format(self.agent_id, msg))

    def _loginfo(self, msg):
        rospy.loginfo("[{}/dispatcher]: {}".format(self.agent_id, msg))

# =====================================================================
# === Function to start the dispatcher
# =====================================================================


def start_dispatcher(cls):
    # Get agent id
    aid = rospy.get_param('~agent_id')

    # Get the source directory for all handlers
    source_dir = rospy.get_param('~source_dir')

    # Get name and type of obligation handlers
    ohandlers = ast.literal_eval(rospy.get_param('~ohandlers', '{}'))

    # Get the parameters for the obligation handlers
    oparams = ast.literal_eval(rospy.get_param('~ohandler_params', '{}'))

    # Get name and type of resource handlers
    rhandlers = ast.literal_eval(rospy.get_param('~rhandlers', '{}'))

    # Get the parameters for the obligation handlers
    rparams = ast.literal_eval(rospy.get_param('~rhandler_params', '{}'))

    # Get the type of comm handler
    ch_type = rospy.get_param('~chandler', '')

    # Get the parameters for the obligation handlers
    cparams = ast.literal_eval(rospy.get_param('~chandler_params', '{}'))

    # Create the dispatcher
    dispatcher = cls(aid)

    # Add the comm handler
    dispatcher._add_comm_handler(
        'comm', ch_type, directory=source_dir, **cparams)

    # Add all obligation handlers
    for oh_id, oh_type in ohandlers.iteritems():
        dispatcher._add_obligation_handler(oh_id, oh_type, directory=source_dir,
                                           **oparams.get(oh_id, {}))

    # Add all result handlers
    for rh_id, rh_type in rhandlers.iteritems():
        dispatcher._add_resource_handler(rh_id, rh_type, directory=source_dir,
                                         **rparams.get(rh_id, {}))

    # Wait until rospy shutdown
    rospy.spin()
