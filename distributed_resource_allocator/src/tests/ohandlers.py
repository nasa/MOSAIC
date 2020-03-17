#!/usr/bin/python

'''
# =====================================================================
# Obligation handler for testing
#
# Author: Marc Sanchez Net
# Date:   05/15/2019
# Copyright (c) 2019, Jet Propulsion Laboratory.
# =====================================================================
'''

# Patch imports
from defaults_scaffolding import patch_imports
patch_imports()

# PDRA imports
from pdra.obligation_handlers import PdraObligationHandlerAction
import pdra.msg as msg

# =====================================================================
# === Valid for all resources in the test
# =====================================================================

class TestObligationHandler(PdraObligationHandlerAction):

    @property
    def action_type_cls(self):
        return msg.RequestResourceAction

    @property
    def action_result_cls(self):
        return msg.RequestResourceResult

    @property
    def action_feedback_cls(self):
        return msg.RequestResourceFeedback

    def serialize_goal_request(self, goal):
        return goal.request

class TestChainedObligationHandler(PdraObligationHandlerAction):

    @property
    def action_type_cls(self):
        return msg.RequestChainAction

    @property
    def action_result_cls(self):
        return msg.RequestChainResult

    @property
    def action_feedback_cls(self):
        return msg.RequestChainFeedback

    def serialize_goal_request(self, goal):
        return goal.request
