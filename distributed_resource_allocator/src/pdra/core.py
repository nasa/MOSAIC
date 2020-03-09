#!/usr/bin/python

'''
# =====================================================================
# PDRA core classes. It includes the ``Dispatchable`` interface,
# ``Obligation`` and ``Result`` classes.
#
# Author: Marc Sanchez Net
# Date:   01/29/2019
# Copyright (c) 2019, Jet Propulsion Laboratory.
# =====================================================================
'''

from copy import deepcopy
from datetime import datetime
import itertools
import json
import uuid

# PDRA imports
from pdra.utils import now

# =====================================================================
# === PDRA version
# =====================================================================

__version__ = '2.0'
__release__ = '2.0a'

# =====================================================================
# === Global variables
# =====================================================================

_default_priority = 5

# =====================================================================
# === Dispatchable Interface Implementation
# =====================================================================

class Dispatchable(object):
    """ Dispatchable interface. It should be subclassed.

        :ivar string: Version - PDRA version
        :ivar UUID:   uid - Dispatchable unique identifier
        :ivar float:  creation_time - POSIX UTC timestamp as float
        :ivar string: req_agent - ID of the agent requesting this dispatchable
        :ivar int:    priority - Lower is higher priority
    """

    # List of parameters to serialize to/from JSON
    # Use slots to save memory
    __slots__ = ['version', 'uid', 'creation_time', 'req_agent', 'priority',
                 '_serialize_attrs', 'type', 'resource_id', 'TTL', 'dependents',
                 'predecessors']

    def __init__(self, version=__version__, uid=None, creation_time=None,
                 priority=_default_priority, req_agent=None,
                 resource_id=None, TTL=float('inf'), dependents=None,
                 predecessors=None, **kwargs):
        """ Class constructor """
        # Get passed values
        self.type          = None
        self.version       = version
        self.uid           = uid
        self.creation_time = creation_time
        self.priority      = priority
        self.req_agent     = req_agent
        self.resource_id   = resource_id
        self.TTL           = TTL
        self.dependents    = dependents
        self.predecessors  = predecessors

        # Compute list of attributes to serialize. By definition these are the
        # the slots of this class an its parents
        attrs = [cls.__slots__ for cls in self.__class__.mro() if cls is not object]
        self._serialize_attrs = list(itertools.chain(*attrs))
        self._serialize_attrs.remove('_serialize_attrs')

        # If null provide default values
        if uid is None:           self.uid = self._new_uid()
        if creation_time is None: self.creation_time = now()
        if predecessors is None:  self.predecessors = []

    @property
    def valid(self):
        """ Returns True if this dispatchable is valid """
        return (self.req_agent is not None) and \
               (self.type is not None) and \
               (self.resource_id is not None) and \
               (now()-self.creation_time < self.TTL)

    @staticmethod
    def _new_uid():
        """ Generate a new unique identifier for this dispatchable based on
            the host ID, sequence number, and current time

            :return UUID: Unique identifier
        """
        return str(uuid.uuid1())

    def to_json(self, **kwargs):
        """ Serialize this dispatchable object to JSON. Only class slots
            will be serialized by default

            :param kwargs: Keyword arguments of ``json.dumps``
            :return: String, output of json.dumps
        """
        # Create dictionary to serialize
        d = {k: getattr(self, k) for k in self._serialize_attrs}

        # Dump to json
        return json.dumps(d, **kwargs)

    @classmethod
    def from_json(cls, str_json, **kwargs):
        """ Load a dispatchable from a JSON encoded string. Only class
            slots will be initialized by default.

            :param json: String JSON encoded
            :param kwargs: Keyword arguments of ``json.loads``
            :return: Dispatchable class instance
        """
        # Load dictionary from JSON
        d = json.loads(str_json, **kwargs)

        # Create new empty instance
        return cls(**d)

    def __str__(self):
        """ Generic string format.
            :return string: Class name
        """
        return '<{}>'.format(self.__class__.__name__)

    def __repr__(self):
        """ Generic class representation.
            :return string: Class name and UID
        """
        return '<{} {}>'.format(self.__class__.__name__, self.uid)

# =====================================================================
# === Obligation Implementation
# =====================================================================

class Obligation(Dispatchable):
    """ A PDRA obligation

        :ivar: See ``Dispatchable``
        :ivar string: resource_id - ID of the resource required by this obligation
        :ivar string: parameters - JSON encoded parameters for the obligation
    """
    __slots__ = ['parameters']

    def __init__(self, resource_id=None, parameters=None, **kwargs):
        # Call parent constructor
        super(Obligation, self).__init__(**kwargs)

        # Set the variables for this obligation
        self.type        = 'Obligation'
        self.resource_id = resource_id
        self.parameters  = parameters

    @classmethod
    def from_result(cls, result, resource_id, parameters):
        # Get relevant attrs from obligation
        attrs = {k: getattr(result, k) for k in result._serialize_attrs}

        # Make a deep copy
        ini_attrs = deepcopy(attrs)

        # Remove attributes that should not be inherited
        attrs.pop('srv_agent')
        attrs.pop('resource_id')

        # Update the TTL with the time left
        t_now = now()
        attrs['TTL'] -= (t_now - attrs['creation_time'])
        attrs['creation_time'] = t_now

        # Create dependent obligation.
        obl = cls(resource_id=resource_id, parameters=parameters, **attrs)

        # Add the information about the predecessor
        obl.predecessors.append(ini_attrs)

        return obl

# =====================================================================
# === Result Implementation
# =====================================================================

class Result(Dispatchable):
    """ A PDRA result

            :ivar: See ``Dispatchable``
            :ivar string: srv_agent - ID of the agent that created this result
            :ivar string: values - JSON encoded results
        """
    __slots__ = ['srv_agent', 'values']

    def __init__(self, srv_agent=None, values=None, **kwargs):
        # Call parent constructor
        super(Result, self).__init__(**kwargs)

        # Set the variables for this obligation
        self.type         = 'Result'
        self.srv_agent    = srv_agent
        self.values       = values

    @classmethod
    def from_obligation(cls, obligation, srv_agent, values):
        # Get relevant attrs from obligation
        attrs = {k: getattr(obligation, k) for k in obligation._serialize_attrs}

        # Update the TTL with the time left
        attrs['TTL'] -= (now() - attrs['creation_time'])

        # Create result
        return cls(srv_agent, values, **attrs)

# =====================================================================
# === MAIN for testing only
# =====================================================================

if __name__ == '__main__':
    a = {'resource_id': 1, 'parameters': [1,2]}
    s = json.dumps(a)
    ob = Obligation.from_json(s)
    print(ob.uid)
    print(ob.parameters)
    print(ob._serialize_attrs)
