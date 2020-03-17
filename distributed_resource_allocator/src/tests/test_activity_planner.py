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

"""

import json
import defaults_scaffolding as def_vals
import rospy
from std_msgs.msg import String


def activity_planner(rate):

    # Open publisher to required topic
    pub_activity = rospy.Publisher('/all/autonomy/activity_plans', String,
                                   queue_size=def_vals.PUB_QUEUE_SIZE)

    pub_problem = rospy.Publisher('/all/distware/problem_description', String,
                                  queue_size=def_vals.PUB_QUEUE_SIZE)

    activity_plans = {
        "Plans": {
            "node_91": {
                "tasks": [
                    {"id": "3", "name": "process", "start_time": 30.0,
                     "duration": 30.0,
                     "params": {"agent": "node_91", "dependents": ["store"]}},
                    {"id": "0", "name": "img", "start_time": 0.0,
                        "duration": 10.0, "params": {"agent": "node_91"}},
                    {"id": "1", "name": "nav", "start_time": 10.0,
                        "duration": 20.0, "params": {"agent": "node_91"}},
                    {"id": "2", "name": "drive", "start_time": 30.0,
                        "duration": 30.0, "params": {"agent": "node_91"}}
                ]
            },
            "node_92": {
                "tasks": [
                    {"id": "0", "name": "img", "start_time": 0.0,
                        "duration": 10.0, "params": {"agent": "node_92"}},
                    {"id": "1", "name": "nav", "start_time": 10.0,
                        "duration": 20.0, "params": {"agent": "node_92"}},
                    {"id": "2", "name": "drive", "start_time": 30.0,
                        "duration": 30.0, "params": {"agent": "node_92"}},
                    {"id": "3", "name": "process", "start_time": 30.0,
                     "duration": 30.0,
                     "params": {"agent": "node_92", "dependents": ["store"]}}
                ]
            },
            "base_station": {
                "tasks": []
            }
        }
    }

    problem_description = {
        "Options": {},
        "Tasks": {
            "ProductsSize": {
                "img": 2.,
                "nav": 0.1,
                "drive": 0.1,
            },
            "TaskReward": {
                "img": 0.,
                "nav": 0.0,
                "drive": 0.,
            },
            "DomainSpecific": {}
        },
        "Time": {
            "TimeHorizon": 60
        },
        "CostFunction": {
            "energy": 1.,
            "total_task_reward": 0.0,
            "total_time": 0
        },
    }

    # Rate at which world plan is published
    rate_ctrl = rospy.Rate(rate)

    # Do until ROS finishes
    while not rospy.is_shutdown():

        # Publish a string mindlessly
        pub_activity.publish(json.dumps(activity_plans, sort_keys=True))
        pub_problem.publish(json.dumps(problem_description, sort_keys=True))

        # Wait for a while
        rate_ctrl.sleep()


if __name__ == "__main__":
    # Start ROS node
    rospy.init_node('activity_planner_test', anonymous=True)

    rate = rospy.get_param('~rate', def_vals.PUB_RATE)

    activity_planner(rate)
