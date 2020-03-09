"""
 Copyright 2020 by California Institute of Technology.  ALL RIGHTS RESERVED.
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

"""
Sample server to expose MOSAIC schedulers as a network service.
To run on localhost with 10 workers and 120s timeout
gunicorn -w 10 -b 127.0.0.1:4000 -t 120 mosaic_schedulers.common.utilities.MOSAIC_server:app
"""

#!/usr/bin/python

from mosaic_schedulers.schedulers.ti_milp import MOSAICTISolver as ti_solver
from mosaic_schedulers.schedulers.ti_milp_heft import Scheduler as ti_heft_solver
from mosaic_schedulers.schedulers.tv_milp import MOSAICSolver as tv_solver
import json
from flask import Flask, request
import argparse
import sys
import os
app = Flask(__name__)

welcome_get_string = "Welcome to MOSAIC! This system replies to POST requests."


@app.route("/", methods=['GET', 'POST'])
def rac_server():
    if request.method == 'GET':
        return welcome_get_string
    else:
        JSONinput = request.get_data()
        time_limit = 30
        TIHScheduler = ti_solver.JSONSolver(
            JSONinput, solver='CPLEX', TimeLimit=time_limit)
        TIHSchedule = TIHScheduler.schedule()

        if TIHSchedule is not None:  # If the problem is feasible
            replyStatus = 200
        else:
            replyStatus = 204
        response = app.response_class(
            response=TIHSchedule,
            status=replyStatus,
            mimetype='application/json'
        )
        return response


@app.route("/ti/", methods=['GET', 'POST'])
def rac_server_ti():
    if request.method == 'GET':
        return welcome_get_string
    else:
        JSONinput = request.get_json()
        time_limit = 30
        TIHScheduler = ti_solver.JSONSolver(
            JSONinput, solver='CPLEX', TimeLimit=time_limit)
        TIHSchedule = TIHScheduler.schedule()

        if TIHSchedule is not None:  # If the problem is feasible
            replyStatus = 200
        else:
            replyStatus = 204
        response = app.response_class(
            response=TIHSchedule,
            status=replyStatus,
            mimetype='application/json'
        )
        return response


@app.route("/ti_heft/", methods=['GET', 'POST'])
def rac_server_ti_heft():
    if request.method == 'GET':
        return welcome_get_string
    else:
        JSONinput = request.get_data()
        time_limit = 30
        TIHScheduler = ti_heft_solver(
            JSONinput, solver='CPLEX', TimeLimit=time_limit)
        TIHSchedule = TIHScheduler.schedule()

        if TIHSchedule is not None:  # If the problem is feasible
            replyStatus = 200
        else:
            replyStatus = 204
        response = app.response_class(
            response=TIHSchedule,
            status=replyStatus,
            mimetype='application/json'
        )
        return response


@app.route("/tv/", methods=['GET', 'POST'])
def rac_server_tv():
    if request.method == 'GET':
        return welcome_get_string
    else:
        JSONinput = request.get_data()
        time_limit = 60
        TIHScheduler = tv_solver.JSONSolver(
            JSONinput, solver='CPLEX', TimeLimit=time_limit)
        TIHSchedule = TIHScheduler.schedule()

        if TIHSchedule is not None:  # If the problem is feasible
            replyStatus = 200
        else:
            replyStatus = 204
        response = app.response_class(
            response=TIHSchedule,
            status=replyStatus,
            mimetype='application/json'
        )
        return response


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Provide a server interface for the MOSAIC solver.',
                                     epilog='For full access to gunicorn options, run gunicorn [options] MOSAIC_server:app from the command line.')
    parser.add_argument('--bind', '-b', default='127.0.0.1:4000',
                        help='The socket to bind (passed to gunicorn).')
    parser.add_argument('--workers', '-w', default=10,
                        help='Number of workers for server (passed to gunicorn).')

    args = parser.parse_args()
    os.system(
        "gunicorn -w {} -b {} -t 120 mosaic_schedulers.common.utilities.MOSAIC_server:app".format(args.workers, args.bind))
    sys.exit(0)
