#!/usr/bin/python

'''
# =====================================================================
# Resource handler for testing purposes
#
# Author: Marc Sanchez Net
# Date:   05/15/2019
# Copyright (c) 2019, Jet Propulsion Laboratory.
# =====================================================================
'''

# Patch imports
import json
import pdra.msg as msg
from pdra.resource_handlers import PdraResourceHandlerAction
from defaults_scaffolding import patch_imports
patch_imports()

# PDRA imports

# =====================================================================
# === Valid for all resources in the test
# =====================================================================


class TestResourceHandler(PdraResourceHandlerAction):

    @property
    def action_type_cls(self):
        return msg.RequestResourceAction

    @property
    def action_goal_cls(self):
        return msg.RequestResourceGoal

    @property
    def action_feedback_cls(self):
        return msg.RequestResourceFeedback

    def fill_goal_info(self, goal):
        goal.request = self.resource_id
        return goal

    def serialize_goal_results(self, goal_result):
        return goal_result.result

    def log_request_done(self, goal_result):
        res_info = json.loads(goal_result.result)["statistics"]

        res_info["name"] = "{}:{}".format(
            self.current_obligation.req_agent, self.resource_id)
        res_info["id"] = res_info["name"]

        self.status_log_publisher.publish(json.dumps(res_info))
